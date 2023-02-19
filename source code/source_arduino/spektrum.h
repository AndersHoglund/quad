
// Spektrum channel order
#define THRO 0
#define AILE 1
#define ELEV 2
#define RUDD 3
#define GEAR 4
#define AUX1 5
#define AUX2 6
#define AUX3 7
#define AUX4 8
#define AUX5 9

// Only available at 22ms frame rate, not at 11ms.
#define AUX6 10
#define AUX7 11

// Only support DSMx SERIALRX_SPEKTRUM2048:
#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEK_FRAME_SIZE                      16
#define SPEK_CHAN_SHIFT                       3
#define SPEK_CHAN_MASK                     0x07
#define SPEKTRUM_NEEDED_FRAME_INTERVAL       10
#define SPEKTRUM_BAUDRATE                115200
