#include <usb_cam/rockchip_mjpeg.h>
#include <ros/ros.h>
#include <stdexcept>
#include <cstring>
#ifdef USB_CAM_ROCKCHIP_MEDIA
#include <rk_mpi.h>
#include <mpp_buffer.h>
#include <mpp_frame.h>
#include <mpp_packet.h>
#include <mpp_meta.h>
#include <mpp_task.h>
#include <RgaApi.h>
#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <cerrno>

namespace usb_cam {
struct RockchipMjpeg::State {
  MppCtx context;
  MppApi *api;
  MppBufferGroup group;
  MppBuffer input, output;
  MppFrame frame;
  int width, height;
  std::size_t capacity;
  unsigned frames;
  State(int w, int h): context(NULL), api(NULL), group(NULL), input(NULL),
      output(NULL), frame(NULL), width(w), height(h), capacity(w*h*3), frames(0) {}
  ~State() {
    if (context) { api->reset(context); mpp_destroy(context); }
    if (frame) mpp_frame_deinit(&frame);
    if (input) mpp_buffer_put(input);
    if (output) mpp_buffer_put(output);
    if (group) mpp_buffer_group_put(group);
  }
};
static void require_mpp(MPP_RET result, const char *operation) {
  if (result != MPP_OK) throw std::runtime_error(std::string(operation)+" failed: "+std::to_string(result));
}
static void sync_input(MppBuffer buffer, bool begin) {
  struct dma_buf_sync sync = {};
  sync.flags=DMA_BUF_SYNC_WRITE | (begin ? DMA_BUF_SYNC_START : DMA_BUF_SYNC_END);
  if (ioctl(mpp_buffer_get_fd(buffer),DMA_BUF_IOCTL_SYNC,&sync)<0 && errno!=ENOTTY)
    throw std::runtime_error("MPP input DMA sync failed");
}
RockchipMjpeg::RockchipMjpeg(int width, int height): state_(new State(width,height)) {
  try {
    if (width%4 || height%2) throw std::runtime_error("MPP/RGA requires width multiple of 4, even height");
    require_mpp(mpp_create(&state_->context,&state_->api),"mpp_create");
    ROS_INFO("MPP context created");
    require_mpp(mpp_init(state_->context,MPP_CTX_DEC,MPP_VIDEO_CodingMJPEG),"mpp_init MJPEG");
    ROS_INFO("MPP MJPEG initialized");
    RK_S64 timeout=500; // Bounded decode wait, milliseconds; never block shutdown indefinitely.
    require_mpp(state_->api->control(state_->context,MPP_SET_OUTPUT_TIMEOUT,&timeout),"MPP timeout");
    MppFrameFormat format=MPP_FMT_YUV420SP;
    require_mpp(state_->api->control(state_->context,MPP_DEC_SET_OUTPUT_FORMAT,&format),"MPP NV12");
    require_mpp(mpp_buffer_group_get_internal(&state_->group,MPP_BUFFER_TYPE_DRM),"MPP DRM group");
    ROS_INFO("MPP DRM group created");
    require_mpp(mpp_buffer_get(state_->group,&state_->input,state_->capacity),"MPP input allocation");
    const int stride=(width+15)&~15, rows=(height+15)&~15;
    require_mpp(mpp_buffer_get(state_->group,&state_->output,stride*rows*4),"MPP output allocation");
    require_mpp(mpp_frame_init(&state_->frame),"MPP frame");
    mpp_frame_set_buffer(state_->frame,state_->output);
    if (c_RkRgaInit()!=0) throw std::runtime_error("RGA init failed; check /dev/rga permissions and ABI");
    ROS_INFO("camera backend=mpp_rga: hardware MJPEG -> NV12 -> RGA RGB8 (BT.601 full range)");
  } catch (...) { delete state_; state_=NULL; throw; }
}
RockchipMjpeg::~RockchipMjpeg() { delete state_; }
bool RockchipMjpeg::decode(const void *jpeg, std::size_t length, char *rgb) {
  State &s=*state_;
  if (length<4 || length>s.capacity) {
    ROS_ERROR_THROTTLE(2,"MPP invalid JPEG payload length: %zu",length); return false;
  }
  sync_input(s.input,true);
  std::memcpy(mpp_buffer_get_ptr(s.input),jpeg,length);
  sync_input(s.input,false);
  MppPacket packet=NULL;
  require_mpp(mpp_packet_init_with_buffer(&packet,s.input),"MPP packet");
  mpp_packet_set_length(packet,length);
  // MPP 1.0.0 MJPEG uses the advanced task queue, not decode_put_packet.
  MppTask task=NULL;
  MPP_RET result=s.api->poll(s.context,MPP_PORT_INPUT,static_cast<MppPollType>(500));
  if (result==MPP_OK) result=s.api->dequeue(s.context,MPP_PORT_INPUT,&task);
  if (result==MPP_OK && task) {
    mpp_task_meta_set_packet(task,KEY_INPUT_PACKET,packet);
    mpp_task_meta_set_frame(task,KEY_OUTPUT_FRAME,s.frame);
    result=s.api->enqueue(s.context,MPP_PORT_INPUT,task);
  } else if (result==MPP_OK) result=MPP_NOK;
  MppFrame decoded=NULL;
  if (result==MPP_OK) result=s.api->poll(s.context,MPP_PORT_OUTPUT,static_cast<MppPollType>(500));
  task=NULL;
  if (result==MPP_OK) result=s.api->dequeue(s.context,MPP_PORT_OUTPUT,&task);
  if (result==MPP_OK && task) mpp_task_meta_get_frame(task,KEY_OUTPUT_FRAME,&decoded);
  if (result!=MPP_OK || !decoded) {
    s.api->reset(s.context); mpp_packet_deinit(&packet);
    ROS_ERROR_THROTTLE(2,"MPP decode failed/timed out: %d; frame discarded",result); return false;
  }
  const int width=mpp_frame_get_width(decoded), height=mpp_frame_get_height(decoded);
  const int hs=mpp_frame_get_hor_stride(decoded), vs=mpp_frame_get_ver_stride(decoded);
  const MppFrameFormat format=mpp_frame_get_fmt(decoded);
  bool valid=width==s.width && height==s.height && format==MPP_FMT_YUV420SP &&
      !mpp_frame_get_errinfo(decoded) && !mpp_frame_get_discard(decoded);
  if (valid) {
    rga_info_t src={},dst={};
    src.fd=mpp_buffer_get_fd(mpp_frame_get_buffer(decoded)); src.mmuFlag=1;
    rga_set_rect(&src.rect,0,0,width,height,hs,vs,RK_FORMAT_YCbCr_420_SP);
    dst.fd=-1; dst.virAddr=rgb; dst.mmuFlag=1;
    dst.color_space_mode=yuv2rgb_mode1; // Legacy RGA API reads CSC from destination.
    rga_set_rect(&dst.rect,0,0,width,height,width,height,RK_FORMAT_RGB_888);
    const int rc=c_RkRgaBlit(&src,&dst,NULL);
    valid=rc==0;
    if (!valid) ROS_ERROR_THROTTLE(2,"RGA conversion failed: %d; frame discarded",rc);
    if (valid && ++s.frames%300==1)
      ROS_INFO("MPP/RGA frame %u: %dx%d NV12 stride=%dx%d -> RGB8",s.frames,width,height,hs,vs);
  } else ROS_ERROR_THROTTLE(2,"MPP invalid frame %dx%d format=%x; discarded",width,height,format);
  require_mpp(s.api->enqueue(s.context,MPP_PORT_OUTPUT,task),"MPP output task recycle");
  task=NULL;
  require_mpp(s.api->poll(s.context,MPP_PORT_INPUT,static_cast<MppPollType>(500)),"MPP input recycle poll");
  require_mpp(s.api->dequeue(s.context,MPP_PORT_INPUT,&task),"MPP input recycle dequeue");
  require_mpp(s.api->enqueue(s.context,MPP_PORT_INPUT,task),"MPP input recycle enqueue");
  mpp_packet_deinit(&packet);
  return valid;
}
}
#else
namespace usb_cam {
struct RockchipMjpeg::State {};
RockchipMjpeg::RockchipMjpeg(int,int):state_(NULL) {
  throw std::runtime_error("mpp_rga requested but usb_cam was built without MPP/RGA libraries");
}
RockchipMjpeg::~RockchipMjpeg() {}
bool RockchipMjpeg::decode(const void*,std::size_t,char*) { return false; }
}
#endif
