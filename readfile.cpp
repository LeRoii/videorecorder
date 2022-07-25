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



int main(int argc, char **argv)
{
    // int img = open("/home/nx/savedvideo/11.yuv", O_RDONLY);
    int img = open(argv[1], O_RDONLY);
    if(img == -1)
        printf("Failed to open file for rendering\n");

    int imgbufsize = 640*514*2;
    unsigned char *imgbuf = (unsigned char*)malloc(imgbufsize);

    int framecnt = 0;
    while(1)
    {
        int cnt = read(img, imgbuf, imgbufsize);
        printf("read %d bytes\n", cnt);
        if(imgbufsize != cnt)
        {
            printf("read end, exit\n");
            return 0;
        }

        framecnt++;
        printf("framecnt:%d\n",framecnt);

        typedef std::chrono::duration<int, std::ratio<86400>> Days;

        std::chrono::system_clock::duration* dur_ptr=reinterpret_cast<std::chrono::system_clock::duration*>(&imgbuf[0]);
        auto dur=*dur_ptr; 
        auto days = std::chrono::duration_cast<Days>(dur);
        dur -= days;
        auto hours = std::chrono::duration_cast<std::chrono::hours>(dur);
        dur -= hours;
        auto minutes = std::chrono::duration_cast<std::chrono::minutes>(dur);
        dur -= minutes;
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(dur);
        dur -= seconds;
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(dur);
        dur -= milliseconds;
        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(dur);
        dur -= microseconds;
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(dur);
            
        std::cout<<hours.count()+8<<":"
                <<minutes.count()<<":"
                <<seconds.count()<<":"
                <<milliseconds.count()<<":"
                <<microseconds.count()<<":"
                <<nanoseconds.count()<<std::endl;
        
        std::string timestr = std::to_string(hours.count()+8)+":"+std::to_string(minutes.count())+":"+std::to_string(seconds.count())\
        +":"+std::to_string(milliseconds.count())+":"+std::to_string(microseconds.count())+":"+std::to_string(nanoseconds.count());

        for(int i=0;i<10;i++)
        {
            printf("[%d] byte:%#x,", i, imgbuf[i]);
        }
        printf("\n");

        cv::Mat mt = cv::Mat(514,640,CV_8UC2, imgbuf);
        cv::Mat mtshow;
        cv::cvtColor(mt, mtshow, cv::COLOR_YUV2RGB_YUYV);
        // cv::imwrite("mtshow.png", mtshow);

        cv::putText(mtshow, timestr, cv::Point(20,20), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,255,0), 1);

        cv::imshow("1", mtshow);
        cv::waitKey(0);


    }
    
    
    
    // cv::Mat mt = cv::Mat(514,640,CV_8UC2, imgbuf);
    // cv::Mat mtshow;
    // cv::cvtColor(mt, mtshow, cv::COLOR_YUV2RGB_UYVY);
    // cv::imwrite("mtshow.png", mtshow);

    return 0;
}