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

struct mdf_logger {
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

extern "C" mdf_logger_err_t mdf_logger_open(const char* path, int can_fd, int compress,
                                            mdf_logger_handle_t* out) {
    if (!path || !out) return MDF_LOGGER_EINVAL;

    auto h = new mdf_logger();
    h->writer = MdfFactory::CreateMdfWriterEx(MdfWriterType::MdfBusLogger);
    if (!h->writer) { delete h; return MDF_LOGGER_EINIT; }

    h->writer->BusType(MdfBusType::CAN);
    h->writer->StorageType(can_fd ? MdfStorageType::MlsdStorage   // CAN FD typical
                                  : MdfStorageType::FixedLengthStorage);
    h->writer->MaxLength(can_fd ? 64 : 8);
    h->writer->CompressData(compress != 0);

    if (!h->writer->Init(path)) { delete h->writer; delete h; return MDF_LOGGER_EIO; }
    if (!h->writer->CreateBusLogConfiguration()) { delete h->writer; delete h; return MDF_LOGGER_EINIT; }

    // Locate the CAN_DataFrame group created by the convenience call
    MdfFile* file = h->writer->GetFile();
    DataGroupList dgs;
    file->DataGroups(dgs);
    if (dgs.empty()) { delete h->writer; delete h; return MDF_LOGGER_EINIT; }
    h->dg = dgs.back();
    h->cg = h->dg->GetChannelGroup("CAN_DataFrame");
    if (!h->cg) { delete h->writer; delete h; return MDF_LOGGER_EINIT; }

    // Start the measurement and internal queue writer thread
    (void)h->writer->InitMeasurement();
    h->writer->StartMeasurement(now_ns());
    h->started = true;

    *out = h;
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
    if (len > sizeof(f->data)) len = sizeof(f->data);
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
