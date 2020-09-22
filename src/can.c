#include "can.h"
#include "abstractCAN.h"
#include "libopencm3/cm3/nvic.h"
#include <libopencm3/cm3/cortex.h>

#include <stddef.h>

uint16_t id_offset = 0;

static const struct abst_can can_settings =
{
    .can_num = 1,  /* Number of CAN (1, 2). */
    .ttcm = false, /* Time triggered communication mode */
    .abom = true,  /* Automatic bus-off management */
    .awum = false, /* Automatic wakeup mode. */
    .nart = false, /* No automatic retransmission */
    .rflm = false, /* Receive FIFO locked mode. */
    .txfp = false, /* Transmit FIFO priority */
    .sjw = CAN_BTR_SJW_1TQ, /* Resynchronization time quanta jump width. */
    .ts1 = CAN_BTR_TS1_3TQ, /* Time segment 1 time quanta width. */
    .ts2 = CAN_BTR_TS2_4TQ, /* Time segment 2 time quanta width. */
    .brp = 12,           /* Baud rate prescaler */
    .loopback = false,   /* Loopback mode */
    .silent = false      /* Silent mode */
};

static struct abst_can_filter_32_bit CAN_SETUP_COMMANDS = 
{
    .filter_id = 0, /* Filter ID */
    .id1 = 200,     /* First message ID to match. Increasing by **id_offset** while initializtion */
    .id2 = 200,     /* Second message ID to match. Used for brodcast */
    .fifo = 0,      /* FIFO ID. */
    .enable = true  /* Enable Filter */
};

static struct abst_can_filter_32_bit CAN_SET_DES_VALUE_COMMANDS = 
{
    .filter_id = 1, /* Filter ID */
    .id1 = 100,     /* First message ID to match. Increasing by **id_offset** while initializtion*/
    .id2 = 100,     /* Second message ID to match. Used for brodcast */
    .fifo = 0,      /* FIFO ID. */
    .enable = true  /* Enable Filter */
};

static struct abst_can_filter_32_bit CAN_LOG_REQUEST_COMMANDS = 
{
    .filter_id = 2,  /* Filter ID */
    .id1 = 800,      /* First message ID to match. Increasing by **id_offset** while initializtion */
    .id2 = 800,      /* Second message ID to match. Used for brodcast */
    .fifo = 1,       /* FIFO ID. */
    .enable = true   /* Enable Filter */
};

bool can_bus_init(void)
{  
    enum abst_errors init_status = abst_can_init(&can_settings);
    if (init_status != ABST_OK)
        return false;
    
    CAN_SETUP_COMMANDS.id1         += id_offset;
    CAN_SET_DES_VALUE_COMMANDS.id1 += id_offset;
    CAN_LOG_REQUEST_COMMANDS.id1   += id_offset;
    
    abst_can_init_filter_32_bit(&CAN_SETUP_COMMANDS);
    
    abst_can_init_filter_32_bit(&CAN_SET_DES_VALUE_COMMANDS);
    
    abst_can_init_filter_32_bit(&CAN_LOG_REQUEST_COMMANDS);
    
    /* Enable CAN interrupts. */
    cm_enable_interrupts();
    
    nvic_enable_irq(NVIC_USB_HP_CAN_TX_IRQ);
    nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
    
    can_enable_irq(CAN1, CAN_IER_FMPIE0);
    can_enable_irq(CAN1, CAN_IER_FMPIE1);
    
    return true;
}

// Interrupt handler
void usb_lp_can_rx0_isr(void)
{
    uint8_t fifo = 0;
    if (abst_can_get_fifo_pending(1, 0))
        fifo = 0;
    else if (abst_can_get_fifo_pending(1, 1)) 
        fifo = 1;
    
    uint32_t id = 0;
    bool ext = 0;
    bool rtr = 0;
    uint8_t fmi = 0;
    uint8_t length = 0;
    uint8_t data[64];
    
    can_receive(CAN1,
                fifo,   /* FIFO */
                true,   /* Release the FIFO automatically after coping data out. */
                &id,    /* Message ID */
                &ext,   /* The message ID is extended */
                &rtr,   /* Request of transmission */
                &fmi,   /* ID of the matched filter */
                &length,/* Length of message payload */
                data,   /* Message payload data */
                NULL);  /* Pointer to store the message timestamp */
    
    if (fmi == 0) 
        change_pid_settings(data, length);
    else if (fmi == 1)
        set_desired_value(data, length);
//     else if (fmi == 2)
        
}
