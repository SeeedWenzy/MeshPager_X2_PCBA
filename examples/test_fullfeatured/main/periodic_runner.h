#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file periodic_runner.h
 * @brief Generic periodic-job runner.
 *
 * Each registered job runs in its own FreeRTOS task that loops:
 *   lazy init (once) -> step (if enabled) -> wait period_ms (wake early on command).
 * Period and enable state are mutable at runtime via the console commands in
 * periodic_runner.c (`period list/set/enable/disable/run`).
 */

typedef esp_err_t (*runner_init_fn)(void *ctx);    /**< lazy, idempotent chip-level init */
typedef esp_err_t (*runner_step_fn)(void *ctx);    /**< one bounded work cycle */
typedef void      (*runner_suspend_fn)(void *ctx); /**< optional teardown on disable */

typedef struct {
    /* ---- caller-provided definition ---- */
    const char       *name;             /**< unique job name (string literal) */
    runner_init_fn    init_fn;          /**< may be NULL */
    runner_step_fn    step_fn;          /**< required */
    runner_suspend_fn suspend_fn;       /**< optional; powers the peripheral down */
    uint32_t          default_period_ms;/**< initial period */
    bool              enabled;          /**< run on boot? */
    uint32_t          stack_bytes;      /**< per-job task stack */
    UBaseType_t       priority;         /**< per-job task priority */
    void             *ctx;              /**< opaque, passed to all callbacks */

    /* ---- runner-managed runtime state ---- */
    uint32_t     period_ms;     /**< current period (mutable) */
    TaskHandle_t task;          /**< job task handle */
    esp_err_t    last_result;   /**< last step/init result */
    uint32_t     run_count;     /**< step invocations so far */
    bool         inited;        /**< init_fn has succeeded */
    bool         prev_enabled;  /**< previous-loop enable state (transition detect) */
    bool         run_once;      /**< set by `period run` to force one extra step */
} runner_job_t;

/**
 * @brief Initialize the runner registry (call once before register/start).
 */
esp_err_t periodic_runner_init(void);

/**
 * @brief Register a job. The definition is copied into the registry; name and
 *        ctx pointers must outlive registration (use string literals / statics).
 */
esp_err_t periodic_runner_register(const runner_job_t *def);

/**
 * @brief Spawn the task for every registered job.
 */
void periodic_runner_start_all(void);

/**
 * @brief Look up a registered job by name (case-insensitive).
 * @return job pointer or NULL if not found.
 */
runner_job_t *periodic_runner_find(const char *name);

/**
 * @brief Clamp a period to the configured minimum (CONFIG_RUNNER_MIN_PERIOD_MS).
 */
uint32_t periodic_runner_clamp_period(uint32_t ms);

/**
 * @brief Register the `period` console commands. Call after esp_console is ready.
 */
void period_cmd_register_all(void);

#ifdef __cplusplus
}
#endif
