#include "mdf_shim.h"
#include <mdf/mdffactory.h>
#include <mdf/mdfwriter.h>
#include <mdf/mdffile.h>
#include <mdf/idatagroup.h>
#include <mdf/ichannelgroup.h>
#include <mdf/canmessage.h>
#include <vector>
#include <string>
#include <time.h>

using namespace mdf;

struct  mdf_logger {
    MdfWriter*       writer = nullptr;
    IDataGroup*      dg     = nullptr;
    IChannelGroup*   cg     = nullptr;
    bool             started = false;
};

static uint64_t now_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

extern "C" mdf_logger_err_t mdf_logger_open(const char* path, int can_fd, int compress, mdf_logger_handle_t* out) {
    if (!path || !out) return MDF_LOGGER_EINVAL;

    auto* logger = new mdf_logger();
    logger->writer = MdfFactory::CreateMdfWriterEx(MdfWriterType::MdfBusLogger);
    if (!logger->writer) { delete logger; return MDF_LOGGER_EINIT; }

    logger->writer->BusType(MdfBusType::CAN);
    logger->writer->StorageType(can_fd ? MdfStorageType::MlsdStorage
                                  : MdfStorageType::FixedLengthStorage);
    logger->writer->MaxLength(can_fd ? 64 : 8);
    logger->writer->CompressData(compress != 0);

    if (!logger->writer->Init(path)) { delete logger->writer; delete logger; return MDF_LOGGER_EIO; }
    if (!logger->writer->CreateBusLogConfiguration()) { delete logger->writer; delete logger; return MDF_LOGGER_EINIT; }

    // Locate the CAN_DataFrame group created by the convenience call
    MdfFile* file = logger->writer->GetFile();
    DataGroupList dgs;
    file->DataGroups(dgs);
    if (dgs.empty()) { delete logger->writer; delete logger; return MDF_LOGGER_EINIT; }
    logger->dg = dgs.back();
    logger->cg = logger->dg->GetChannelGroup("CAN_DataFrame");
    if (!logger->cg) { delete logger->writer; delete logger; return MDF_LOGGER_EINIT; }

    // Start the measurement and internal queue writer thread
    (void)logger->writer->InitMeasurement();
    logger->writer->StartMeasurement(now_ns());
    logger->started = true;

    *out = logger;
    return MDF_LOGGER_OK;
}

extern "C" mdf_logger_err_t mdf_logger_write(mdf_logger_handle_t h,
                                             const mdf_can_frame_t* f,
                                             uint64_t tns) {
    if (!h || !h->writer || !h->cg || !f) return MDF_LOGGER_EINVAL;

    CanMessage msg;
    msg.MessageId(f->id);
    msg.ExtendedId(f->is_extended != 0);
    msg.Rtr(f->rtr != 0);
    msg.Edl(f->is_fd != 0);
    msg.Brs(f->brs != 0);
    msg.Dir(f->is_tx != 0);

    size_t len = f->is_fd ? CanMessage::DlcToLength(f->dlc) : f->dlc;
    //if (len > sizeof(f->data)) len = sizeof(f->data);
    std::vector<uint8_t> bytes(f->data, f->data + len);
    msg.DataBytes(bytes); // sets DLC appropriately for CAN/CAN-FD

    const uint64_t ts = tns ? tns : now_ns();
    h->writer->SaveCanMessage(*h->cg, ts, msg);
    return MDF_LOGGER_OK;
}

extern "C" mdf_logger_err_t mdf_logger_flush(mdf_logger_handle_t h) {
    // No explicit flush API; queue is flushed by the writer thread.
    // We keep this for future expansion. Return OK.
    return h ? MDF_LOGGER_OK : MDF_LOGGER_EINVAL;
}

extern "C" mdf_logger_err_t mdf_logger_close(mdf_logger_handle_t h) {
    if (!h || !h->writer) return MDF_LOGGER_EINVAL;
    if (h->started) h->writer->StopMeasurement(now_ns());
    (void)h->writer->FinalizeMeasurement();
    delete h->writer;     // CreateMdfWriterEx -> delete
    delete h;
    return MDF_LOGGER_OK;
}
