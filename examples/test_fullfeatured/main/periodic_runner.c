/*
 * periodic_runner.c
 *
 * Generic periodic-job runner. Each job runs in its own FreeRTOS task that
 * loops: lazy init -> step (if enabled) -> wait period (wake early on command).
 * Period and enable state are mutable at runtime through the `period_*`
 * console commands registered at the bottom of this file.
 */

#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "esp_console.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "periodic_runner.h"

static const char *TAG = "RUNNER";

#define RUNNER_MAX_JOBS 12

static runner_job_t s_jobs[RUNNER_MAX_JOBS];
static int s_job_count = 0;
static SemaphoreHandle_t s_lock = NULL;

/* ------------------------------------------------------------------ */
/* Helpers                                                            */
/* ------------------------------------------------------------------ */

uint32_t periodic_runner_clamp_period(uint32_t ms)
{
#if defined(CONFIG_RUNNER_MIN_PERIOD_MS) && CONFIG_RUNNER_MIN_PERIOD_MS > 0
    if (ms < CONFIG_RUNNER_MIN_PERIOD_MS) {
        ms = CONFIG_RUNNER_MIN_PERIOD_MS;
    }
#endif
    return ms;
}

static void wake_job(runner_job_t *job)
{
    if (job != NULL && job->task != NULL) {
        xTaskNotifyGive(job->task);
    }
}

/* ------------------------------------------------------------------ */
/* Per-job task                                                       */
/* ------------------------------------------------------------------ */

static void runner_task(void *arg)
{
    runner_job_t *job = (runner_job_t *)arg;
    esp_err_t r;

    /* seed transition detector */
    xSemaphoreTake(s_lock, portMAX_DELAY);
    job->prev_enabled = job->enabled;
    xSemaphoreGive(s_lock);

    ESP_LOGI(TAG, "job \"%s\" started (period=%lums enabled=%d)",
             job->name, (unsigned long)job->period_ms, job->enabled);

    while (true) {
        bool enabled;
        bool run_once;
        uint32_t period;
        bool need_init;
        bool need_suspend;

        /* snapshot desired state + detect transitions */
        xSemaphoreTake(s_lock, portMAX_DELAY);
        enabled       = job->enabled;
        run_once      = job->run_once;
        job->run_once = false;
        period        = job->period_ms;
        need_init     = !job->inited && (enabled || run_once);
        need_suspend  = job->inited && job->prev_enabled && !enabled;
        xSemaphoreGive(s_lock);

        /* lazy init (first time we actually need the hardware) */
        if (need_init) {
            r = (job->init_fn != NULL) ? job->init_fn(job->ctx) : ESP_OK;
            xSemaphoreTake(s_lock, portMAX_DELAY);
            job->inited      = (r == ESP_OK);
            job->last_result = r;
            xSemaphoreGive(s_lock);
            if (r != ESP_OK) {
                ESP_LOGE(TAG, "init \"%s\" failed: %s", job->name, esp_err_to_name(r));
            } else {
                ESP_LOGI(TAG, "init \"%s\" ok", job->name);
            }
        }

        /* teardown on enable->disable transition; force re-init on re-enable */
        if (need_suspend) {
            if (job->suspend_fn != NULL) {
                job->suspend_fn(job->ctx);
            }
            xSemaphoreTake(s_lock, portMAX_DELAY);
            job->inited = false;
            xSemaphoreGive(s_lock);
            ESP_LOGI(TAG, "suspend \"%s\"", job->name);
        }

        /* one bounded step when enabled or explicitly poked */
        if (job->inited && (enabled || run_once)) {
            r = job->step_fn(job->ctx);
            xSemaphoreTake(s_lock, portMAX_DELAY);
            job->last_result = r;
            job->run_count++;
            xSemaphoreGive(s_lock);
            if (r != ESP_OK) {
                ESP_LOGW(TAG, "step \"%s\" returned %s", job->name, esp_err_to_name(r));
            }
        }

        xSemaphoreTake(s_lock, portMAX_DELAY);
        job->prev_enabled = job->enabled;
        xSemaphoreGive(s_lock);

        /* wait for the period; commands wake us early so edits apply promptly */
        uint32_t wait = period ? period : portMAX_DELAY;
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait));
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                         */
/* ------------------------------------------------------------------ */

esp_err_t periodic_runner_init(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    return ESP_OK;
}

esp_err_t periodic_runner_register(const runner_job_t *def)
{
    if (s_lock == NULL || def == NULL || def->step_fn == NULL || def->name == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_job_count >= RUNNER_MAX_JOBS) {
        ret = ESP_ERR_NO_MEM;
    } else {
        for (int i = 0; i < s_job_count; i++) {
            if (strcasecmp(s_jobs[i].name, def->name) == 0) {
                ret = ESP_ERR_INVALID_STATE;
                break;
            }
        }
        if (ret == ESP_OK) {
            runner_job_t *job = &s_jobs[s_job_count];
            *job = *def;                       /* copy definition */
            job->period_ms     = def->default_period_ms;
            job->task          = NULL;
            job->last_result   = ESP_OK;
            job->run_count     = 0;
            job->inited        = false;
            job->prev_enabled  = false;
            job->run_once      = false;
            s_job_count++;
        }
    }
    xSemaphoreGive(s_lock);
    return ret;
}

void periodic_runner_start_all(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < s_job_count; i++) {
        runner_job_t *job = &s_jobs[i];
        if (job->task != NULL) {
            continue;
        }
        if (job->stack_bytes == 0) {
            job->stack_bytes = 4096;
        }
        if (job->priority == 0) {
            job->priority = 5;
        }
        char name[configMAX_TASK_NAME_LEN];
        snprintf(name, sizeof(name), "run_%s", job->name);
        /* Allocate each job's stack in PSRAM (CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM
         * is set). ~50 KB of job stacks would otherwise consume internal RAM and
         * starve DMA-capable heap (e.g. the SDMMC bounce/internal DMA buffer
         * fails to allocate under concurrent WiFi/BLE/audio/sdcard load). */
        BaseType_t ok = xTaskCreateWithCaps(runner_task, name,
                                            job->stack_bytes / sizeof(StackType_t),
                                            job, job->priority, &job->task,
                                            MALLOC_CAP_SPIRAM);
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "failed to create task for \"%s\"", job->name);
            job->task = NULL;
        }
    }
    xSemaphoreGive(s_lock);
}

runner_job_t *periodic_runner_find(const char *name)
{
    runner_job_t *found = NULL;
    if (name == NULL) {
        return NULL;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < s_job_count; i++) {
        if (strcasecmp(s_jobs[i].name, name) == 0) {
            found = &s_jobs[i];
            break;
        }
    }
    xSemaphoreGive(s_lock);
    return found;
}

/* ================================================================== */
/* Console commands                                                    */
/* ================================================================== */

static int period_list_cmd(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    printf("%-10s %-7s %-9s %-9s %-8s %s\n",
           "job", "en", "period", "runs", "init", "last");
    printf("--------------------------------------------------------------\n");
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < s_job_count; i++) {
        runner_job_t *j = &s_jobs[i];
        printf("%-10s %-7s %-9lu %-9lu %-8s %s\n",
               j->name,
               j->enabled ? "yes" : "no",
               (unsigned long)j->period_ms,
               (unsigned long)j->run_count,
               j->inited ? "ok" : "-",
               j->inited ? esp_err_to_name(j->last_result) : "-");
    }
    xSemaphoreGive(s_lock);
    return 0;
}

static struct {
    struct arg_str *name;
    struct arg_int *ms;
    struct arg_end *end;
} period_set_args;

static int period_set_real(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&period_set_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, period_set_args.end, argv[0]);
        return 1;
    }
    runner_job_t *job = periodic_runner_find(period_set_args.name->sval[0]);
    if (job == NULL) {
        printf("unknown job: %s\n", period_set_args.name->sval[0]);
        return 1;
    }
    uint32_t ms = (uint32_t)period_set_args.ms->ival[0];
    ms = periodic_runner_clamp_period(ms);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    job->period_ms = ms;
    xSemaphoreGive(s_lock);
    wake_job(job);
    printf("\"%s\" period set to %lu ms\n", job->name, (unsigned long)ms);
    return 0;
}

static struct {
    struct arg_str *name;
    struct arg_end *end;
} period_enable_args;

static int period_enable_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&period_enable_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, period_enable_args.end, argv[0]);
        return 1;
    }
    runner_job_t *job = periodic_runner_find(period_enable_args.name->sval[0]);
    if (job == NULL) {
        printf("unknown job: %s\n", period_enable_args.name->sval[0]);
        return 1;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    job->enabled = true;
    xSemaphoreGive(s_lock);
    wake_job(job);
    printf("\"%s\" enabled\n", job->name);
    return 0;
}

static int period_disable_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&period_enable_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, period_enable_args.end, argv[0]);
        return 1;
    }
    runner_job_t *job = periodic_runner_find(period_enable_args.name->sval[0]);
    if (job == NULL) {
        printf("unknown job: %s\n", period_enable_args.name->sval[0]);
        return 1;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    job->enabled = false;
    xSemaphoreGive(s_lock);
    wake_job(job);
    printf("\"%s\" disabled\n", job->name);
    return 0;
}

static int period_run_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&period_enable_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, period_enable_args.end, argv[0]);
        return 1;
    }
    runner_job_t *job = periodic_runner_find(period_enable_args.name->sval[0]);
    if (job == NULL) {
        printf("unknown job: %s\n", period_enable_args.name->sval[0]);
        return 1;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    job->run_once = true;
    xSemaphoreGive(s_lock);
    wake_job(job);
    printf("\"%s\" run requested\n", job->name);
    return 0;
}

static void register_period_cmd(const char *command, const char *help,
                                esp_console_cmd_func_t func, void *argtable)
{
    const esp_console_cmd_t cmd = {
        .command = command,
        .help = help,
        .hint = NULL,
        .func = func,
        .argtable = argtable,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void period_cmd_register_all(void)
{
    period_set_args.name = arg_str1(NULL, NULL, "<name>", "job name (see period_list)");
    period_set_args.ms   = arg_int1(NULL, NULL, "<ms>", "new period in ms");
    period_set_args.end  = arg_end(2);

    period_enable_args.name = arg_str1(NULL, NULL, "<name>", "job name");
    period_enable_args.end  = arg_end(1);

    register_period_cmd("period_list", "List all jobs with state/period", &period_list_cmd, NULL);
    register_period_cmd("period_set", "Set a job's period: period_set <name> <ms>", &period_set_real, &period_set_args);
    register_period_cmd("period_enable", "Enable a job: period_enable <name>", &period_enable_cmd, &period_enable_args);
    register_period_cmd("period_disable", "Disable a job: period_disable <name>", &period_disable_cmd, &period_enable_args);
    register_period_cmd("period_run", "Run one step now: period_run <name>", &period_run_cmd, &period_enable_args);

    ESP_LOGI(TAG, "period commands registered: period_list / period_set / period_enable / period_disable / period_run");
}
