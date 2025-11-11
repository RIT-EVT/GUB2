#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct mdf_logger* mdf_logger_handle_t;

    typedef struct {
        uint32_t id;          // 11/29-bit CAN ID (lower bits)
        uint8_t  dlc;         // 0..15
        uint8_t  is_extended; // 1=29-bit ID
        uint8_t  is_fd;       // 1=CAN FD
        uint8_t  is_tx;       // 1=transmitted, 0=received
        uint8_t  brs;         // FD bit-rate switch
        uint8_t  rtr;         // remote frame
        uint8_t  reserved;    // align
        uint8_t  data[64];    // payload
    } mdf_can_frame_t;

    typedef enum {
        MDF_LOGGER_OK    = 0,
        MDF_LOGGER_EINVAL= -1,
        MDF_LOGGER_EINIT = -2,
        MDF_LOGGER_EIO   = -3,
        MDF_LOGGER_ESTATE= -4
    } mdf_logger_err_t;

    mdf_logger_err_t mdf_logger_open(const char* path, int can_fd, int compress,
                                     mdf_logger_handle_t* out);
    mdf_logger_err_t mdf_logger_write(mdf_logger_handle_t h,
                                      const mdf_can_frame_t* frame,
                                      uint64_t timestamp_ns);
    mdf_logger_err_t mdf_logger_flush(mdf_logger_handle_t h); // optional
    mdf_logger_err_t mdf_logger_close(mdf_logger_handle_t h);

#ifdef __cplusplus
}
#endif
