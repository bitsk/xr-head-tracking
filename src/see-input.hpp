// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// Included after Sample and the existing timestamp/log helpers. This consumer
// never opens IIO or an SDSP device: the selected device-io producer owns both
// IMU streams, the DMA buffers and the same ownership lock as the IIO route.
class DirectImuInput {
public:
    ~DirectImuInput() { close(); }
    void close() { if(fd_>=0) { ::close(fd_);fd_=-1; } }
    bool connect()
    {
        const char *path=std::getenv("XR_SEE_IMU_SOCKET");
        sockaddr_un address{};address.sun_family=AF_UNIX;
        if(!path || std::strncmp(path,"/run/",5) || std::strlen(path)>=sizeof(address.sun_path)) return false;
        std::strcpy(address.sun_path,path);
        fd_=socket(AF_UNIX,SOCK_SEQPACKET|SOCK_CLOEXEC,0);
        if(fd_<0 || ::connect(fd_,reinterpret_cast<sockaddr *>(&address),sizeof(address))) return false;
        ucred peer{};socklen_t length=sizeof(peer);
        if(getsockopt(fd_,SOL_SOCKET,SO_PEERCRED,&peer,&length) || peer.uid!=geteuid()) return false;
        report("see_input",{},"source=direct period_us="+std::to_string(period_)+" notification=sns_llcm_poll");
        return true;
    }
    int fd() const { return fd_; }
    bool read_batch(std::vector<Sample> &batch,const std::array<int,3> &axes)
    {
        for(unsigned i=0;i<128;++i) {
            xr::see::Packet packet{};
            const auto n=recv(fd_,&packet,sizeof(packet),MSG_DONTWAIT|MSG_TRUNC);
            if(n<0 && (errno==EAGAIN || errno==EINTR)) return true;
            if(n!=sizeof(packet) || packet.kind>1 || packet.requested_period_us!=period_) {
                report("see_input",failure(EPROTO),"closed, malformed or mismatched source");return false;
            }
            const auto kind=packet.kind;
            std::uint64_t time{};float xyz[3]{};
            if(!xr::see::decode(packet,kind==1,time,xyz) || time<=last_[kind] ||
                time>boottime_ns()+1000000 || boottime_ns()>time+1000000000) {
                report("see_input",failure(EPROTO),"invalid sample clock/order/value");return false;
            }
            auto counter=xr::see::get32(packet.event,12);
            auto delta=counter>=counter_[kind] ? counter-counter_[kind] : UINT32_MAX-counter_[kind]+counter;
            if(counter_[kind] && (!delta || delta>=0x80000000u)) return false;
            if(counter_[kind] && delta>1) gaps_[kind]+=delta-1;
            counter_[kind]=counter;last_[kind]=time;++samples_[kind];
            Sample sample{time,{},kind==1};
            for(unsigned axis=0;axis<3;++axis)
                sample.value[axis]=xyz[std::abs(axes[axis])-1]*(axes[axis]<0 ? -1.0f : 1.0f);
            batch.push_back(sample);
        }
        return true;
    }
    void summary() const {
        report("see_summary",{},"accel_samples="+std::to_string(samples_[0])+" gyro_samples="+
            std::to_string(samples_[1])+" accel_counter_gaps="+std::to_string(gaps_[0])+
            " gyro_counter_gaps="+std::to_string(gaps_[1])+" requested_period_us="+std::to_string(period_));
    }
private:
    int fd_=-1;static constexpr unsigned period_=1250;
    std::uint64_t last_[2]{},samples_[2]{},gaps_[2]{};
    std::uint32_t counter_[2]{};
};
