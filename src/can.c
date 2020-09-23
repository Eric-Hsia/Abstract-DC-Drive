#include "can.h"

#include "common_defs.h"

#include <abstractCAN.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>
#include <abstractLOG.h>
#include <stddef.h>

struct abst_pin can_RX = {
    .port = ABST_GPIOA,
    .num = 11,
    .mode = ABST_MODE_AF,
    .af_dir = ABST_AF_INPUT,
    .otype = ABST_OTYPE_PP,
    .speed = ABST_OSPEED_50MHZ,
    .pull_up_down = ABST_PUPD_NONE,
    .is_inverse = false
};

struct abst_pin can_TX = {
    .port = ABST_GPIOA,
    .num = 12,
    .mode = ABST_MODE_AF,
    .af_dir = ABST_AF_OUTPUT,
    .otype = ABST_OTYPE_PP,
    .speed = ABST_OSPEED_50MHZ,
    .pull_up_down = ABST_PUPD_NONE,
    .is_inverse = false
};

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
    .brp = 450, //19,           /* Baud rate prescaler */
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

/* Log CAN IDs. Increasing by **id_offset** while initializtion */
static uint32_t LOG_SPEED_CAN_ID = 1000;
static uint32_t LOG_POSITION_CAN_ID = 1100;
static uint32_t LOG_CURRENT_CAN_ID = 1200;
static uint32_t LOG_TEMPERATURE_CAN_ID = 1300;

static void send_log(uint8_t data[], uint8_t N);

bool can_bus_init(void)
{  
    abst_gpio_init(&can_RX);
    abst_digital_write(&can_RX, 1);
    abst_gpio_init(&can_TX);
    
        /* Enable CAN interrupts. */
    cm_enable_interrupts();
    
    nvic_enable_irq(NVIC_USB_HP_CAN_TX_IRQ);
    nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
    nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 1);
    
    
    
    enum abst_errors init_status = abst_can_init(&can_settings);
    if (init_status != ABST_OK) {
        abst_log("Init CAN failed\n");
        return false;
    }
    
    can_enable_irq(CAN1, CAN_IER_FMPIE0);
    can_enable_irq(CAN1, CAN_IER_FMPIE1);
    
    while (CAN_IER(CAN1) == 0);
    
    CAN_SETUP_COMMANDS.id1         += id_offset;
    CAN_SET_DES_VALUE_COMMANDS.id1 += id_offset;
    CAN_LOG_REQUEST_COMMANDS.id1   += id_offset;
    
    LOG_SPEED_CAN_ID += id_offset;
    LOG_POSITION_CAN_ID += id_offset;
    LOG_CURRENT_CAN_ID += id_offset;
    LOG_TEMPERATURE_CAN_ID += id_offset;
    
    abst_can_init_filter_32_bit(&CAN_SETUP_COMMANDS);
    
    abst_can_init_filter_32_bit(&CAN_SET_DES_VALUE_COMMANDS);
    
    abst_can_init_filter_32_bit(&CAN_LOG_REQUEST_COMMANDS);

//     can_filter_id_mask_32bit_init(
//             0,     /* Filter ID */
//             0,     /* CAN ID */
//             0,     /* CAN ID mask */
//             0,     /* FIFO assignment (here: FIFO0) */
//             true); /* Enable the filter. */
    
    abst_log("Init CAN success\n");
    return true;
}

// Interrupt handler
void usb_lp_can_rx0_isr(void)
{
    abst_log("\n\n\n\nCAN interrupt\n");
    abst_logf("Fifo pending: %i\n", abst_can_get_fifo_pending(1, 0));

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
    
    abst_logf("CAN receive: Id: %i, fil: %i, len: %i\n", (int)id, (int)fmi, (int)length);
    
    if (id == CAN_SETUP_COMMANDS.id1) 
        change_pid_settings(data, length);
    else if (id == CAN_SET_DES_VALUE_COMMANDS.id1)
        set_desired_value(data, length);
    else if (id == CAN_LOG_REQUEST_COMMANDS.id1)
        send_log(data, length);
}

static void send_log(uint8_t data[], uint8_t N)
{
    abst_logf("Got log request: %i\n", (int)data[0]);
    
    if (N != 1)
        return; // Unknown data format
    
    int32_t msg;
    uint32_t can_id;
    switch (data[0]) {
        case LOG_POSITION:
            msg = get_encoder_value();
            can_id = LOG_POSITION_CAN_ID;
            break;
        case LOG_SPEED:
            msg = regulator_get_fd_speed();
            can_id = LOG_SPEED_CAN_ID;
            break;
        case LOG_CURRENT:
            msg = get_current_value();
            can_id = LOG_CURRENT_CAN_ID;
            break;
        case LOG_TEMPERATURE:
            /** Fall througnt. TODO */
        default:
            return; // Unknown type
    }
    can_transmit(   CAN1,
                    can_id,         /* (EX/ST)ID: CAN ID */
                    false,          /* IDE: CAN ID extended? */
                    false,          /* RTR: Request transmit? */
                    sizeof(msg),    /* DLC: Data length */
                    &msg);
}
