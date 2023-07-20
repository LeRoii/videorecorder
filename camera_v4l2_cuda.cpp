/*
 * Copyright (c) 2016-2019, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <signal.h>
#include <poll.h>
#include <chrono>
#include <vector>
#include <thread>
#include <mutex>


#include <dirent.h>
#include <sys/statfs.h>
#include <sys/time.h>
#include <iomanip>
#include <sys/timeb.h>
#include <signal.h>

#include "test.pb.h"


#include "NvEglRenderer.h"
#include "NvUtils.h"
#include "NvCudaProc.h"
#include "nvbuf_utils.h"

#include "camera_v4l2_cuda.h"


extern int uart_read(int fd, unsigned char * buf, unsigned int len, unsigned int wait_usec);
extern int  set_port_attr (
              int fd,
              int  baudrate,          // B1200 B2400 B4800 B9600 .. B115200 B230400
              int  databit,           // 5, 6, 7, 8
              const char *stopbit,    //  "1", "1.5", "2"
              char parity,            // N(o), O(dd), E(ven)
              int vtime,
              int vmin );
extern int  init_serial(const char* serial_name,
				int baudrate,
				int databit,
				const char* stopbit,
				char parity);
extern int onRcvGPSMsg(unsigned char* msg, int len);

static int CROP_TOP = 283;
static int CROP_LEFT = 640;

static bool quit = false;

std::mutex serialMtx;

int openedFile = -1;

std::string dataRootPath = "/home/nvidia/savedvideo/";
std::string dataFullPath;

bool writevideo = true;

context_t ctx;

using namespace std;

uint64_t createtimestamp()
{
	auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    uint64_t* ptr=reinterpret_cast<uint64_t*>(&duration);
	uint64_t  tmp=*ptr;
	return tmp;
}

// using namespace itas109;
// class mySlot : public has_slots<>
// {
// public:
//     mySlot(CSerialPort *sp)
//     {
//         recLen = -1;
//         p_sp = sp;

//         std::string pbfilepath;
//         std::time_t tt = std::chrono::system_clock::to_time_t (std::chrono::system_clock::now());
//         std::stringstream ss;
//         ss << std::put_time(std::localtime(&tt), "%F-%H-%M-%S");
//         std::string str = dataFullPath.c_str()+ss.str()+".db";
//         ss.str("");
//         ss << str;
//         ss >> pbfilepath;  
//         serialOutput.open(pbfilepath, std::ios::out | std::ios::binary);
//         if(!serialOutput.is_open())
//         {
//             printf("pb file open failed");
//         }

//         msgcnt = 0;
//     };

//     void OnSendMessage()
//     {
//         static pb::msg_bag bag;
//         // read
//         recLen = p_sp->readAllData(serialdata);
//         if (recLen > 0)
//         {
//             printf("recLen:%d\n", recLen);
//             for(int i=0;i<recLen;i++)
//             {
//                 printf( "  0x%02X\n", serialdata[ i ]);
//             }

//             serialMtx.lock();
//             pmsg = bag.add_msgs();
//             serialMtx.unlock();

//             pmsg->set_timestamp(createtimestamp());
//             pmsg->set_type(pb::serial_msg_Msg_type_GPS);
//             pmsg->set_len(recLen);
//             // char pbdata[90]={i%256};
//             pmsg->set_body(serialdata,recLen);

//             msgcnt++;

//             if(msgcnt == 200*60*30)
//             {
//                 msgcnt = 0;
//                 auto str=bag.SerializeAsString();
//                 uint64_t len=str.size();
//                 char* start=reinterpret_cast<char*>(&len);
//                 serialOutput.write(start, 8);
//                 serialOutput.write(str.c_str(), str.length());
//                 serialOutput.close();
//                 bag.clear_msgs();

//                 std::string pbfilepath;
//                 std::time_t tt = std::chrono::system_clock::to_time_t (std::chrono::system_clock::now());
//                 std::stringstream ss;
//                 ss << std::put_time(std::localtime(&tt), "%F-%H-%M-%S");
//                 std::string filenamestr = dataFullPath.c_str()+ss.str()+".db";
//                 ss.str("");
//                 ss << filenamestr;
//                 ss >> pbfilepath;  
//                 serialOutput.open(pbfilepath, std::ios::out | std::ios::binary);
//                 printf("pb bag end, open a new pb file");

//             }

//         }
//     };

// private:
//     mySlot(){};
// public:
//     ~mySlot(){serialOutput.close();}

// private:
//     CSerialPort *p_sp;

//     char serialdata[1024];
//     int recLen;
//     pb::serial_msg* pmsg;
    

//     std::fstream serialOutput;
//     int msgcnt;
   
    
// };


static void
print_usage(void)
{
    printf("\n\tUsage: camera_v4l2_cuda [OPTIONS]\n\n"
           "\tExample: \n"
           "\t./camera_v4l2_cuda -d /dev/video0 -s 640x480 -f YUYV -n 30 -c\n\n"
           "\tSupported options:\n"
           "\t-d\t\tSet V4l2 video device node\n"
           "\t-s\t\tSet output resolution of video device\n"
           "\t-f\t\tSet output pixel format of video device (supports only YUYV/YVYU/UYVY/VYUY/GREY/MJPEG)\n"
           "\t-r\t\tSet renderer frame rate (30 fps by default)\n"
           "\t-n\t\tSave the n-th frame before VIC processing\n"
           "\t-c\t\tEnable CUDA aglorithm (draw a black box in the upper left corner)\n"
           "\t-v\t\tEnable verbose message\n"
           "\t-h\t\tPrint this usage\n\n"
           "\tNOTE: It runs infinitely until you terminate it with <ctrl+c>\n");
}

void showAllFiles( const char * dir_name, std::vector<std::string> &files)
{
	// check the parameter !
	if( NULL == dir_name )
	{
		cout<<" dir_name is null ! "<<endl;
		return;
	}
 
	// check if dir_name is a valid dir
	struct stat s;
	lstat( dir_name , &s );
	if( ! S_ISDIR( s.st_mode ) )
	{
		cout<<"dir_name is not a valid directory !"<<endl;
		return;
	}
	
	struct dirent * filename;    // return value for readdir()
 	DIR * dir;                   // return value for opendir()
	dir = opendir( dir_name );
	if( NULL == dir )
	{
		cout<<"Can not open dir "<<dir_name<<endl;
		return;
	}
	cout<<"Successfully opened the dir !"<<endl;
	
	/* read all the files in the dir ~ */
	while( ( filename = readdir(dir) ) != NULL )
	{
		// get rid of "." and ".."
		if( strcmp( filename->d_name , "." ) == 0 || 
			strcmp( filename->d_name , "..") == 0    )
			continue;
		// cout<<filename ->d_name <<endl;
		files.push_back(filename ->d_name);
	}
} 

 size_t GetFileSize(const std::string& file_name){
	std::ifstream in(file_name.c_str());
	in.seekg(0, std::ios::end);
	size_t size = in.tellg();
	in.close();
	return size; //单位是：Byte
}

unsigned long checkAvailable()
{
	struct statfs diskInfo;
	statfs("./", &diskInfo);

	unsigned long long blocksize                = diskInfo.f_bsize;	//每个block里包含的字节数
	unsigned long long totalsize                = blocksize * diskInfo.f_blocks; 	//总的字节数，f_blocks为block的数目

	// printf("Total_size = %llu B                 = %llu KB = %llu MB = %llu GB\n", 
	// 	                                            totalsize, totalsize>>10, totalsize>>20, totalsize>>30);
	
	unsigned long long freeDisk                 = diskInfo.f_bfree * blocksize;	//剩余空间的大小
	unsigned long long availableDisk            = diskInfo.f_bavail * blocksize; 	//可用空间大小
	// printf("Disk_free = %llu MB                 = %llu GB\nDisk_available = %llu MB = %llu GB\n", 
	// 	                                            freeDisk>>20, freeDisk>>30, availableDisk>>20, availableDisk>>30);
	return availableDisk>>20;

}

int rm_dir(std::string dir_full_path) {
    DIR* dirp = opendir(dir_full_path.c_str());
    if(!dirp)
    {
        return -1;
    }
    struct dirent *dir;
    struct stat st;
    while((dir = readdir(dirp)) != NULL)
    {
        if(strcmp(dir->d_name,".") == 0
           || strcmp(dir->d_name,"..") == 0)
        {
            continue;
        }
        std::string sub_path = dir_full_path + '/' + dir->d_name;
        if(lstat(sub_path.c_str(),&st) == -1)
        {
            //Log("rm_dir:lstat ",sub_path," error");
            continue;
        }
        if(S_ISDIR(st.st_mode))
        {
            if(rm_dir(sub_path) == -1) // 如果是目录文件，递归删除
            {
                closedir(dirp);
                return -1;
            }
            rmdir(sub_path.c_str());
        }
        else if(S_ISREG(st.st_mode))
        {
            unlink(sub_path.c_str());     // 如果是普通文件，则unlink
        }
        else
        {
            //Log("rm_dir:st_mode ",sub_path," error");
            continue;
        }
    }
    if(rmdir(dir_full_path.c_str()) == -1)//delete dir itself.
    {
        closedir(dirp);
        return -1;
    }
    closedir(dirp);
    return 0;
}

bool Remove(std::string file_name) {
    std::string file_path = file_name;
    struct stat st;
    if (lstat(file_path.c_str(),&st) == -1) {
        return EXIT_FAILURE;
    }
    if (S_ISREG(st.st_mode)) {
        if (unlink(file_path.c_str()) == -1) {
            return EXIT_FAILURE;
        }
    }
    else if(S_ISDIR(st.st_mode)) {
        if(file_name == "." || file_name == "..") {
            return EXIT_FAILURE;
        }
        if(rm_dir(file_path) == -1)//delete all the files in dir.
        {
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;

}


static bool
parse_cmdline(context_t * ctx, int argc, char **argv)
{
    int c;

    if (argc < 2)
    {
        print_usage();
        exit(EXIT_SUCCESS);
    }

    while ((c = getopt(argc, argv, "d:s:f:r:n:cvh")) != -1)
    {
        switch (c)
        {
            case 'd':
                ctx->cam_devname = optarg;
                break;
            case 's':
                if (sscanf(optarg, "%dx%d",
                            &ctx->cam_w, &ctx->cam_h) != 2)
                {
                    print_usage();
                    return false;
                }
                break;
            case 'f':
                if (strcmp(optarg, "YUYV") == 0)
                    ctx->cam_pixfmt = V4L2_PIX_FMT_YUYV;
                else if (strcmp(optarg, "YVYU") == 0)
                    ctx->cam_pixfmt = V4L2_PIX_FMT_YVYU;
                else if (strcmp(optarg, "VYUY") == 0)
                    ctx->cam_pixfmt = V4L2_PIX_FMT_VYUY;
                else if (strcmp(optarg, "UYVY") == 0)
                    ctx->cam_pixfmt = V4L2_PIX_FMT_UYVY;
                else if (strcmp(optarg, "GREY") == 0)
                    ctx->cam_pixfmt = V4L2_PIX_FMT_GREY;
                else if (strcmp(optarg, "MJPEG") == 0)
                    ctx->cam_pixfmt = V4L2_PIX_FMT_MJPEG;
                else
                {
                    print_usage();
                    return false;
                }
                sprintf(ctx->cam_file, "camera.%s", optarg);
                break;
            case 'r':
                ctx->fps = strtol(optarg, NULL, 10);
                break;
            case 'n':
                ctx->save_n_frame = strtol(optarg, NULL, 10);
                break;
            case 'c':
                ctx->enable_cuda = true;
                break;
            case 'v':
                ctx->enable_verbose = true;
                break;
            case 'h':
                print_usage();
                exit(EXIT_SUCCESS);
                break;
            default:
                print_usage();
                return false;
        }
    }

    return true;
}

static void
set_defaults(context_t * ctx)
{
    memset(ctx, 0, sizeof(context_t));

    ctx->cam_devname = "/dev/video3";
    ctx->cam_fd = -1;
    ctx->cam_pixfmt = V4L2_PIX_FMT_YUYV;
    ctx->cam_w = 1920;
    ctx->cam_h = 1080;
    ctx->frame = 0;
    ctx->save_n_frame = 0;

    ctx->g_buff = NULL;
    ctx->capture_dmabuf = true;
    ctx->renderer = NULL;
    ctx->fps = 30;

    ctx->enable_cuda = false;
    ctx->egl_image = NULL;
    ctx->egl_display = EGL_NO_DISPLAY;

    ctx->enable_verbose = false;
}

static nv_color_fmt nvcolor_fmt[] =
{
    // TODO add more pixel format mapping
    {V4L2_PIX_FMT_UYVY, NvBufferColorFormat_UYVY},
    {V4L2_PIX_FMT_VYUY, NvBufferColorFormat_VYUY},
    {V4L2_PIX_FMT_YUYV, NvBufferColorFormat_YUYV},
    {V4L2_PIX_FMT_YVYU, NvBufferColorFormat_YVYU},
    {V4L2_PIX_FMT_GREY, NvBufferColorFormat_GRAY8},
    {V4L2_PIX_FMT_YUV420M, NvBufferColorFormat_YUV420},
};

static NvBufferColorFormat
get_nvbuff_color_fmt(unsigned int v4l2_pixfmt)
{
    unsigned i;

    for (i = 0; i < sizeof(nvcolor_fmt) / sizeof(nvcolor_fmt[0]); i++)
    {
        if (v4l2_pixfmt == nvcolor_fmt[i].v4l2_pixfmt)
            return nvcolor_fmt[i].nvbuff_color;
    }

    return NvBufferColorFormat_Invalid;
}

static bool
save_frame_to_file(context_t * ctx, struct v4l2_buffer * buf)
{
    int file;

    file = open(ctx->cam_file, O_CREAT | O_WRONLY | O_APPEND | O_TRUNC,
            S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);

    if (-1 == file)
        ERROR_RETURN("Failed to open file for frame saving");

    if (-1 == write(file, ctx->g_buff[buf->index].start,
                ctx->g_buff[buf->index].size))
    {
        close(file);
        ERROR_RETURN("Failed to write frame into file");
    }

    close(file);

    return true;
}

static bool
nvbuff_do_clearchroma (int dmabuf_fd)
{
  NvBufferParams params = {0};
  void *sBaseAddr[3] = {NULL};
  int ret = 0;
  int size;
  unsigned i;

  ret = NvBufferGetParams (dmabuf_fd, &params);
  if (ret != 0)
    ERROR_RETURN("%s: NvBufferGetParams Failed \n", __func__);

  for (i = 1; i < params.num_planes; i++) {
    ret = NvBufferMemMap (dmabuf_fd, i, NvBufferMem_Read_Write, &sBaseAddr[i]);
    if (ret != 0)
      ERROR_RETURN("%s: NvBufferMemMap Failed \n", __func__);

    ret = NvBufferMemSyncForCpu (dmabuf_fd, i, &sBaseAddr[i]);
    if (ret != 0)
      ERROR_RETURN("%s: NvBufferMemSyncForCpu Failed \n", __func__);

    size = params.height[i] * params.pitch[i];
    memset (sBaseAddr[i], 0x80, size);

    ret = NvBufferMemSyncForDevice (dmabuf_fd, i, &sBaseAddr[i]);
    if (ret != 0)
      ERROR_RETURN("%s: NvBufferMemSyncForDevice Failed \n", __func__);

    ret = NvBufferMemUnMap (dmabuf_fd, i, &sBaseAddr[i]);
    if (ret != 0)
      ERROR_RETURN("%s: NvBufferMemUnMap Failed \n", __func__);
  }

  return true;
}

static bool
camera_initialize(context_t * ctx)
{
    struct v4l2_format fmt;

    // Open camera device
    ctx->cam_fd = open(ctx->cam_devname, O_RDWR);
    if (ctx->cam_fd == -1)
        ERROR_RETURN("Failed to open camera device %s: %s (%d)",
                ctx->cam_devname, strerror(errno), errno);

    // Set camera output format
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = ctx->cam_w;
    fmt.fmt.pix.height = ctx->cam_h;
    fmt.fmt.pix.pixelformat = ctx->cam_pixfmt;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    if (ioctl(ctx->cam_fd, VIDIOC_S_FMT, &fmt) < 0)
        ERROR_RETURN("Failed to set camera output format: %s (%d)",
                strerror(errno), errno);

    // Get the real format in case the desired is not supported
    memset(&fmt, 0, sizeof fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(ctx->cam_fd, VIDIOC_G_FMT, &fmt) < 0)
        ERROR_RETURN("Failed to get camera output format: %s (%d)",
                strerror(errno), errno);
    if (fmt.fmt.pix.width != ctx->cam_w ||
            fmt.fmt.pix.height != ctx->cam_h ||
            fmt.fmt.pix.pixelformat != ctx->cam_pixfmt)
    {
        WARN("The desired format is not supported");
        ctx->cam_w = fmt.fmt.pix.width;
        ctx->cam_h = fmt.fmt.pix.height;
        ctx->cam_pixfmt =fmt.fmt.pix.pixelformat;
    }

    struct v4l2_streamparm streamparm;
    memset (&streamparm, 0x00, sizeof (struct v4l2_streamparm));
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl (ctx->cam_fd, VIDIOC_G_PARM, &streamparm);

    INFO("Camera ouput format: (%d x %d)  stride: %d, imagesize: %d, frate: %u / %u",
            fmt.fmt.pix.width,
            fmt.fmt.pix.height,
            fmt.fmt.pix.bytesperline,
            fmt.fmt.pix.sizeimage,
            streamparm.parm.capture.timeperframe.denominator,
            streamparm.parm.capture.timeperframe.numerator);

    return true;
}

static bool
display_initialize(context_t * ctx)
{
    // Create EGL renderer
    // ctx->renderer = NvEglRenderer::createEglRenderer("renderer0",
    //         ctx->cam_w, ctx->cam_h, 0, 0);

    ctx->renderer = NvEglRenderer::createEglRenderer("renderer0",
            640, 480, 0, 0);
    if (!ctx->renderer)
        ERROR_RETURN("Failed to create EGL renderer");
    ctx->renderer->setFPS(ctx->fps);

    if (ctx->enable_cuda)
    {
        // Get defalut EGL display
        ctx->egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        if (ctx->egl_display == EGL_NO_DISPLAY)
            ERROR_RETURN("Failed to get EGL display connection");

        // Init EGL display connection
        if (!eglInitialize(ctx->egl_display, NULL, NULL))
            ERROR_RETURN("Failed to initialize EGL display connection");
    }

    return true;
}

static bool
init_components(context_t * ctx)
{
    if (!camera_initialize(ctx))
        ERROR_RETURN("Failed to initialize camera device");

    // if (!display_initialize(ctx))
    //     ERROR_RETURN("Failed to initialize display");

    INFO("Initialize v4l2 components successfully");
    return true;
}

static bool
request_camera_buff(context_t *ctx)
{
    // Request camera v4l2 buffer
    struct v4l2_requestbuffers rb;
    memset(&rb, 0, sizeof(rb));
    rb.count = V4L2_BUFFERS_NUM;
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rb.memory = V4L2_MEMORY_DMABUF;
    if (ioctl(ctx->cam_fd, VIDIOC_REQBUFS, &rb) < 0)
        ERROR_RETURN("Failed to request v4l2 buffers: %s (%d)",
                strerror(errno), errno);
    if (rb.count != V4L2_BUFFERS_NUM)
        ERROR_RETURN("V4l2 buffer number is not as desired");

    for (unsigned int index = 0; index < V4L2_BUFFERS_NUM; index++)
    {
        struct v4l2_buffer buf;

        // Query camera v4l2 buf length
        memset(&buf, 0, sizeof buf);
        buf.index = index;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_DMABUF;

        if (ioctl(ctx->cam_fd, VIDIOC_QUERYBUF, &buf) < 0)
            ERROR_RETURN("Failed to query buff: %s (%d)",
                    strerror(errno), errno);

        // TODO add support for multi-planer
        // Enqueue empty v4l2 buff into camera capture plane
        buf.m.fd = (unsigned long)ctx->g_buff[index].dmabuff_fd;
        if (buf.length != ctx->g_buff[index].size)
        {
            WARN("Camera v4l2 buf length is not expected");
            ctx->g_buff[index].size = buf.length;
        }

        if (ioctl(ctx->cam_fd, VIDIOC_QBUF, &buf) < 0)
            ERROR_RETURN("Failed to enqueue buffers: %s (%d)",
                    strerror(errno), errno);
    }

    return true;
}

static bool
request_camera_buff_mmap(context_t *ctx)
{
    // Request camera v4l2 buffer
    struct v4l2_requestbuffers rb;
    memset(&rb, 0, sizeof(rb));
    rb.count = V4L2_BUFFERS_NUM;
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rb.memory = V4L2_MEMORY_MMAP;
    if (ioctl(ctx->cam_fd, VIDIOC_REQBUFS, &rb) < 0)
        ERROR_RETURN("Failed to request v4l2 buffers: %s (%d)",
                strerror(errno), errno);
    if (rb.count != V4L2_BUFFERS_NUM)
        ERROR_RETURN("V4l2 buffer number is not as desired");

    for (unsigned int index = 0; index < V4L2_BUFFERS_NUM; index++)
    {
        struct v4l2_buffer buf;

        // Query camera v4l2 buf length
        memset(&buf, 0, sizeof buf);
        buf.index = index;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(ctx->cam_fd, VIDIOC_QUERYBUF, &buf) < 0)
            ERROR_RETURN("Failed to query buff: %s (%d)",
                    strerror(errno), errno);

        ctx->g_buff[index].size = buf.length;
        ctx->g_buff[index].start = (unsigned char *)
            mmap (NULL /* start anywhere */,
                    buf.length,
                    PROT_READ | PROT_WRITE /* required */,
                    MAP_SHARED /* recommended */,
                    ctx->cam_fd, buf.m.offset);
        if (MAP_FAILED == ctx->g_buff[index].start)
            ERROR_RETURN("Failed to map buffers");

        if (ioctl(ctx->cam_fd, VIDIOC_QBUF, &buf) < 0)
            ERROR_RETURN("Failed to enqueue buffers: %s (%d)",
                    strerror(errno), errno);
    }

    return true;
}


static bool
prepare_buffers(context_t * ctx)
{
    NvBufferCreateParams input_params = {0};

    // Allocate global buffer context
    ctx->g_buff = (nv_buffer *)malloc(V4L2_BUFFERS_NUM * sizeof(nv_buffer));
    if (ctx->g_buff == NULL)
        ERROR_RETURN("Failed to allocate global buffer context");

    input_params.payloadType = NvBufferPayload_SurfArray;
    input_params.width = ctx->cam_w;
    input_params.height = ctx->cam_h;
    input_params.layout = NvBufferLayout_Pitch;

    // Create buffer and provide it with camera
    for (unsigned int index = 0; index < V4L2_BUFFERS_NUM; index++)
    {
        int fd;
        NvBufferParams params = {0};

        input_params.colorFormat = get_nvbuff_color_fmt(ctx->cam_pixfmt);
        input_params.nvbuf_tag = NvBufferTag_CAMERA;
        if (-1 == NvBufferCreateEx(&fd, &input_params))
            ERROR_RETURN("Failed to create NvBuffer");

        ctx->g_buff[index].dmabuff_fd = fd;

        if (-1 == NvBufferGetParams(fd, &params))
            ERROR_RETURN("Failed to get NvBuffer parameters");

        if (ctx->cam_pixfmt == V4L2_PIX_FMT_GREY &&
            params.pitch[0] != params.width[0])
                ctx->capture_dmabuf = false;

        // TODO add multi-planar support
        // Currently it supports only YUV422 interlaced single-planar
        if (ctx->capture_dmabuf) {
            if (-1 == NvBufferMemMap(ctx->g_buff[index].dmabuff_fd, 0, NvBufferMem_Read_Write,
                        (void**)&ctx->g_buff[index].start))
                ERROR_RETURN("Failed to map buffer");
        }
    }

    input_params.colorFormat = get_nvbuff_color_fmt(ctx->cam_pixfmt);
    input_params.width = 640;
    input_params.height = 514;
    input_params.nvbuf_tag = NvBufferTag_NONE;
    if (-1 == NvBufferCreateEx(&ctx->store_dmabuf_fd, &input_params))
            ERROR_RETURN("Failed to create store_dmabuf_fd");

    if (-1 == NvBufferMemMap(ctx->store_dmabuf_fd, 0, NvBufferMem_Read_Write,
                        (void**)&ctx->pStoreStart))
                ERROR_RETURN("Failed to map buffer");


    input_params.colorFormat = get_nvbuff_color_fmt(V4L2_PIX_FMT_YUV420M);
    input_params.width = 640;
    input_params.height = 514;
    input_params.nvbuf_tag = NvBufferTag_NONE;
    // Create Render buffer
    if (-1 == NvBufferCreateEx(&ctx->render_dmabuf_fd, &input_params))
        ERROR_RETURN("Failed to create NvBuffer");

    if (ctx->capture_dmabuf) {
        if (!request_camera_buff(ctx))
            ERROR_RETURN("Failed to set up camera buff");
    } else {
        if (!request_camera_buff_mmap(ctx))
            ERROR_RETURN("Failed to set up camera buff");
    }

    INFO("Succeed in preparing stream buffers");
    return true;
}

static bool
start_stream(context_t * ctx)
{
    enum v4l2_buf_type type;

    // Start v4l2 streaming
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(ctx->cam_fd, VIDIOC_STREAMON, &type) < 0)
        ERROR_RETURN("Failed to start streaming: %s (%d)",
                strerror(errno), errno);

    usleep(200);

    INFO("Camera video streaming on ...");
    return true;
}

static bool
stop_stream(context_t * ctx)
{
    enum v4l2_buf_type type;

    // Stop v4l2 streaming
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(ctx->cam_fd, VIDIOC_STREAMOFF, &type))
        ERROR_RETURN("Failed to stop streaming: %s (%d)",
                strerror(errno), errno);

    INFO("Camera video streaming off ...");
    return true;
}


static int cleanup()
{
    stop_stream(&ctx);
    if(ctx.cam_fd > 0)
        close(ctx.cam_fd);

    if (ctx.renderer != NULL)
        delete ctx.renderer;

    if (ctx.egl_display && !eglTerminate(ctx.egl_display))
        printf("Failed to terminate EGL display connection\n");

    if (ctx.g_buff != NULL)
    {
        for (unsigned i = 0; i < V4L2_BUFFERS_NUM; i++) {
            if (ctx.g_buff[i].dmabuff_fd)
                NvBufferDestroy(ctx.g_buff[i].dmabuff_fd);
            if (ctx.cam_pixfmt == V4L2_PIX_FMT_MJPEG)
                munmap(ctx.g_buff[i].start, ctx.g_buff[i].size);
        }
        free(ctx.g_buff);
    }

    NvBufferDestroy(ctx.render_dmabuf_fd);

    // if (error)
    //     printf("App run failed\n");
    // else
        printf("cleanup end\n");

}


static void
signal_handle(int signum)
{
    close(openedFile);
    printf("Quit due to exit command from user!\n");
    quit = true;
    cleanup();
}

static bool
start_capture(context_t * ctx, int core)
{
    // for(auto &str:dirs)
    // {
    // std::cout<<str<<std::endl;
    // }

    struct sigaction sig_action;
    struct pollfd fds[1];
    NvBufferTransformParams transParams;

    // Ensure a clean shutdown if user types <ctrl+c>
    sig_action.sa_handler = signal_handle;
    sigemptyset(&sig_action.sa_mask);
    sig_action.sa_flags = 0;
    sigaction(SIGINT, &sig_action, NULL);

    // Init the NvBufferTransformParams
    memset(&transParams, 0, sizeof(transParams));
    NvBufferRect srcRect, dstRect;
    srcRect.top = CROP_TOP;
    srcRect.left = CROP_LEFT;
    srcRect.width = 640;
    srcRect.height = 514;

    dstRect.top = 0;
    dstRect.left = 0;
    dstRect.width = 640;
    dstRect.height = 514;

    // transParams.transform_flag = NVBUFFER_TRANSFORM_FILTER;
    transParams.transform_flag = NVBUFFER_TRANSFORM_CROP_SRC;
    transParams.transform_filter = NvBufferTransform_Filter_Smart;
    transParams.src_rect = srcRect;
    transParams.dst_rect = dstRect;

    // Enable render profiling information
    // ctx->renderer->enableProfiling();

    fds[0].fd = ctx->cam_fd;
    fds[0].events = POLLIN;

    
    std::string filePath;
    // file = open(ctx->cam_file, O_CREAT | O_WRONLY | O_APPEND | O_TRUNC,
    //         S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);

    while (poll(fds, 1, 5000) > 0 && !quit)
    {
        if (fds[0].revents & POLLIN) {
            struct v4l2_buffer v4l2_buf;

            // Dequeue camera buff
            memset(&v4l2_buf, 0, sizeof(v4l2_buf));
            v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (ctx->capture_dmabuf)
                v4l2_buf.memory = V4L2_MEMORY_DMABUF;
            else
                v4l2_buf.memory = V4L2_MEMORY_MMAP;
            if (ioctl(ctx->cam_fd, VIDIOC_DQBUF, &v4l2_buf) < 0)
                ERROR_RETURN("Failed to dequeue camera buff: %s (%d)",
                        strerror(errno), errno);

            auto stamp = createtimestamp();

            unsigned char* ptr=reinterpret_cast<unsigned char*>(&stamp);

            // for(int i=0;i<8;i++)
            // {
            //     printf("%#x, ", ptr[i]);
            // }

            ctx->frame++;

            if (-1 == NvBufferTransform(ctx->g_buff[v4l2_buf.index].dmabuff_fd, ctx->store_dmabuf_fd,
                            &transParams))
                    ERROR_RETURN("Failed to convert the buffer");

            memcpy(ctx->pStoreStart, ptr, 8);

            // if (ctx->frame == ctx->save_n_frame)
                // save_frame_to_file(ctx, &v4l2_buf);

            // if (-1 == file)
            //     ERROR_RETURN("Failed to open file for frame saving");

            // write file
            if(checkAvailable() < 150)
            {
                std::vector<string> fileNames;
                showAllFiles(dataFullPath.c_str(), fileNames);
                std::sort(fileNames.begin(), fileNames.end());
                Remove(dataFullPath.c_str()+fileNames[0]);
                Remove(dataFullPath.c_str()+fileNames[1]);
            }
            // for(int i=0;i<8;i++)
            // {
            //     printf("%#x, ", ctx->pStoreStart[i]);
            // }
            // printf("\n");

            // printf("ctx->frame:%d\n", ctx->frame);

            if(writevideo)
            {
                if(openedFile == -1 || GetFileSize(filePath) > 104857600*5 )
                {
                    printf("open new file, openedFile =%d\n", openedFile);
                    std::time_t tt = std::chrono::system_clock::to_time_t (std::chrono::system_clock::now());
                    std::stringstream ss;
                    ss << std::put_time(std::localtime(&tt), "%F-%H-%M-%S");
                    std::string str = dataFullPath.c_str()+ss.str()+".yuv";
                    ss.str("");
                    ss << str;
                    ss >> filePath;  

                    if(openedFile != -1)
                    {
                        close(openedFile);
                    }

                    openedFile = open(filePath.c_str(), O_CREAT | O_WRONLY | O_APPEND | O_TRUNC,
                        S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);

                }

                int writesize = 640*514*2;
                
                if (-1 == write(openedFile, ctx->pStoreStart,
                            writesize))
                {
                    printf("write failed\n");
                    close(openedFile);
                    ERROR_RETURN("Failed to write frame into file");
                }
            }
            else
            {
                close(openedFile);
                openedFile = -1;
                // printf("close, openedFile:%d\n", openedFile);
            }
            // close(openedFile);

            // printf("frame:%d, ctx->g_buff[v4l2_buf.index].size:%d\n", ctx->frame, ctx->g_buff[v4l2_buf.index].size);
            // for(int i =0;i<ctx->g_buff[v4l2_buf.index].size/2;i++){
            //     unsigned char tmp= ctx->g_buff[v4l2_buf.index].start[i*2];
            //     ctx->g_buff[v4l2_buf.index].start[i*2]=ctx->g_buff[v4l2_buf.index].start[i*2+1];
            //     ctx->g_buff[v4l2_buf.index].start[i*2+1]=tmp;

            // }
            
            // if(ctx->frame >5)
            // {
            //     close(openedFile);
            //     printf("return\n");
            //  return 0;
            // }

            {
                if (ctx->capture_dmabuf) {
                    // Cache sync for VIC operation
                    NvBufferMemSyncForDevice(ctx->g_buff[v4l2_buf.index].dmabuff_fd, 0,
                            (void**)&ctx->g_buff[v4l2_buf.index].start);
                } else {
                    Raw2NvBuffer(ctx->g_buff[v4l2_buf.index].start, 0,
                             ctx->cam_w, ctx->cam_h, ctx->g_buff[v4l2_buf.index].dmabuff_fd);
                }
            }


            // Enqueue camera buff
            if (ioctl(ctx->cam_fd, VIDIOC_QBUF, &v4l2_buf))
                ERROR_RETURN("Failed to queue camera buffers: %s (%d)",
                        strerror(errno), errno);
        }
    }

    // Print profiling information when streaming stops.
    // ctx->renderer->printProfilingStats();

    if (ctx->cam_pixfmt == V4L2_PIX_FMT_MJPEG)
        delete ctx->jpegdec;

    return true;
}

void serialTH1()
{
    int fd=init_serial("/dev/ttyTHS1", B921600, 8, "1", 'n');
    //int fd=init_serial("/dev/ttyTHS0", B460800, 8, "1", 'e');
    assert(fd>=0);
    unsigned long sum=0, gsum=0, csum=0, wsum=0;	
    int maxSize=512;
    std::string str;
    str.resize(512);
    std::vector<int> warray;
    while(1)
    {         
        ioctl(fd, FIONREAD, &maxSize);
        if(maxSize>str.size())
            str.resize(maxSize);
        int ret=uart_read(fd, reinterpret_cast<unsigned char*>(&str[0]), maxSize, 0);
        if(ret>0){
            printf("rcv No.%d, len %d: ", sum++, ret);
            for(int i=0; i<ret; ++i)
                printf("%#x ", str[i]);
            printf("\n");
            if((ret==140)&&(str[0]==0x55)&&(str[1]=0xaa)) ++csum;
            else
            {
                    ++wsum;
                    warray.push_back(ret);
                    

            }
            gsum+=onRcvGPSMsg(reinterpret_cast<unsigned char*>(&str[0]), ret);
            
            printf("csum %d, gsum %d, wsum %d:", csum, gsum, wsum);
            if(!warray.empty())
            {
                for(auto & i:warray)
                    printf(" %d",i);
            }
            printf(".\n");
                
        }
    }
}

void serialTH0()
{
    // int fd=init_serial("/dev/ttyTHS1", B921600, 8, "1", 'n');
    int fd=init_serial("/dev/ttyTHS0", B460800, 8, "1", 'e');
    assert(fd>=0);
    unsigned long sum=0, gsum=0, csum=0, wsum=0;	
    int maxSize=512;
    std::string str;
    str.resize(512);
    std::vector<int> warray;
    while(1)
    {         
        ioctl(fd, FIONREAD, &maxSize);
        if(maxSize>str.size())
            str.resize(maxSize);
        int ret=uart_read(fd, reinterpret_cast<unsigned char*>(&str[0]), maxSize, 0);
        if(ret>0){
            printf("rcv No.%d, len %d: ", sum++, ret);
            for(int i=0; i<ret; ++i)
                printf("%#x ", str[i]);
            printf("\n");
            if((ret==140)&&(str[0]==0x55)&&(str[1]=0xaa)) ++csum;
            else
            {
                    ++wsum;
                    warray.push_back(ret);
            }
            gsum+=onRcvGPSMsg(reinterpret_cast<unsigned char*>(&str[0]), ret);
            
            printf("csum %d, gsum %d, wsum %d:", csum, gsum, wsum);
            if(!warray.empty())
            {
                for(auto & i:warray)
                    printf(" %d",i);
            }
            printf(".\n");
                
        }
    }
}


int
main(int argc, char *argv[])
{
    if(argc > 2)
    {
        CROP_TOP = stoi(argv[1]);
        CROP_LEFT = stoi(argv[2]);
    }

    // make a new directory
    std::vector<string> fileNames, dirs;
    showAllFiles(dataRootPath.c_str(), fileNames);
    for(auto &str:fileNames)
    {
        if(str.find(".") == std::string::npos)
            dirs.push_back(str);
    }

    int maxfoldername = 0;
    for(auto &str:dirs)
    {
        if(stoi(str) > maxfoldername)
        {
            maxfoldername = stoi(str);
        }
        cout<<str<<endl;
    }

    // std::sort(dirs.begin(), dirs.end());
    // dataFullPath
    dataFullPath = dataRootPath + std::to_string(maxfoldername+1)+"/";
    //    auto newfolderpath = dataRootPath+newfolder;
    if(mkdir(dataFullPath.c_str(), 0777) == 0)
    {
        printf("mk new dir ok\n");
    }


    size_t index = -1;

    int error = 0;

    set_defaults(&ctx);

    init_components(&ctx);
    prepare_buffers(&ctx);
    start_stream(&ctx);

    int core = 0;
    std::thread videoth = std::thread(start_capture, &ctx, core);
    videoth.detach();

    while(1 && !quit)
    {
        char c = getchar();
        // printf("1111\n");
        if(c == 'f')
        {
            writevideo = false;
            printf("%c\n", c);
        }
        else if(c == 't')
        {
            std::vector<string> fileNames, dirs;
            showAllFiles(dataRootPath.c_str(), fileNames);
            for(auto &str:fileNames)
            {
                if(str.find(".") == std::string::npos)
                    dirs.push_back(str);
            }

            int maxfoldername = 0;
            for(auto &str:dirs)
            {
                if(stoi(str) > maxfoldername)
                {
                    maxfoldername = stoi(str);
                }
                cout<<str<<endl;
            }

            // dataFullPath
            dataFullPath = dataRootPath + std::to_string(maxfoldername+1)+"/";
            //    auto newfolderpath = dataRootPath+newfolder;
            if(mkdir(dataFullPath.c_str(), 0777) == 0)
            {
                printf("mkdir ok\n");
            }
            writevideo = true;
        }
            
    }
}
