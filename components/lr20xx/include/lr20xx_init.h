#ifndef LR20xx_INIT__
#define LR20xx_INIT__

#include <stdint.h>
 #include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t (*lr20xx_transfer_t)(uint8_t *p_tx_buf, uint8_t tx_len,uint8_t *p_rx_buf, uint8_t rx_len);

typedef struct
{
    uint32_t    nss_pin;
    uint32_t    rst_pin;
    uint32_t    irq_pin;
    uint32_t    busy_pin;
    uint32_t    rf_sw_pin;

    lr20xx_transfer_t transfer;

} lr20xx_obj_t;

void lr20xx_init(const void* context);

#ifdef __cplusplus
}
#endif

#endif
