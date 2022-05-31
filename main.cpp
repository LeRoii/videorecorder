#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <vector>

#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/statfs.h>




using namespace cv;
using namespace std;
using std::thread;

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

	printf("Total_size = %llu B                 = %llu KB = %llu MB = %llu GB\n", 
		                                            totalsize, totalsize>>10, totalsize>>20, totalsize>>30);
	
	unsigned long long freeDisk                 = diskInfo.f_bfree * blocksize;	//剩余空间的大小
	unsigned long long availableDisk            = diskInfo.f_bavail * blocksize; 	//可用空间大小
	printf("Disk_free = %llu MB                 = %llu GB\nDisk_available = %llu MB = %llu GB\n", 
		                                            freeDisk>>20, freeDisk>>30, availableDisk>>20, availableDisk>>30);
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

string command;

void keboardListener()
{
	while(1)
    {
        cin>>command;
        cout<<"command:"<<command<<endl;
    }
}

int main()
{
	// VideoCapture cap(0);
	std::cout<<cv::getBuildInformation()<<std::endl;
	VideoCapture cap(" v4l2src device=/dev/video0 ! video/x-raw, format=UYVY, width=1920, height=1080, framerate=30/1! videoconvert ! video/x-raw, format=BGR!  appsink", cv::CAP_GSTREAMER);
	// cap.release();
	// cap = VideoCapture(0);
	//gst_testsink.open("appsrc ! queue ! videoconvert ! video/x-raw, format=RGBA ! nvvidconv ! nvoverlaysink ", cv::CAP_GSTREAMER, 0, 3, cv::Size(1280, 720));
	cv::Mat img;

	printf("is opened:%d\n", cap.isOpened());
	

	double fps=0;

	// cap.set(cv::CAP_PROP_FRAME_WIDTH,1280);
	// cap.set(cv::CAP_PROP_FRAME_HEIGHT,720);
	//cap.set(cv::CAP_PROP_FPS,30);

	// thread keyboardListenerTh(keboardListener);
	int cnt = 0;
	// int screenshotCnt = 1;

	// std::cout <<"checkAvailable:" << checkAvailable() << "MB" << endl;

	// std::vector<string> fileNames;
	// showAllFiles("/home/nxsd/savedvideo/", fileNames);
	// std::sort(fileNames.begin(), fileNames.end());
	// Remove("/home/nxsd/savedvideo/"+fileNames[0]);
	// Remove("/home/nxsd/savedvideo/"+fileNames[1]);
	// return 0;

	
	// // std::vector<string> fileNames;
	// // getFileNames("/home/nxsd/savedvideo/", fileNames);
	// std::cout <<"before sort"<<endl;
	// for (const auto &ph : fileNames) {
	// 	std::cout << ph << "\n";
	// }
	// std::cout <<"after sort"<<endl;
	// std::sort(fileNames.begin(), fileNames.end());

	// for (const auto &ph : fileNames) {
	// 	std::cout << ph << "\n";
	// }

	// return 0;


	std::string filePath = "out.avi";
	

	// std::time_t tt = std::chrono::system_clock::to_time_t (std::chrono::system_clock::now());
	// std::stringstream ss;
	// ss << std::put_time(std::localtime(&tt), "%F-%H-%M-%S");
	// std::string str = "/home/nxsd/savedvideo/"+ss.str()+".avi";
	// ss.str("");
	// ss << str;
	// ss >> filePath;

	cv::VideoWriter *writer = nullptr;
	// cv::VideoWriter videoWriter(filePath, CV_FOURCC('M','J','P','G'), 10, cv::Size(1920, 1080));
	while(true)
	{
		if(checkAvailable() < 150)
		{
			std::vector<string> fileNames;
			showAllFiles("/home/nxsd/savedvideo/", fileNames);
			std::sort(fileNames.begin(), fileNames.end());
			Remove("/home/nxsd/savedvideo/"+fileNames[0]);
			Remove("/home/nxsd/savedvideo/"+fileNames[1]);
		}
		if(writer == nullptr || GetFileSize(filePath) > 104857600 )
		{
			std::time_t tt = std::chrono::system_clock::to_time_t (std::chrono::system_clock::now());
			std::stringstream ss;
			ss << std::put_time(std::localtime(&tt), "%F-%H-%M-%S");
			std::string str = "/home/nxsd/savedvideo/"+ss.str()+".avi";
			ss.str("");
			ss << str;
			ss >> filePath;
			if(writer != nullptr)
			{
				writer->release();
			}
			writer = new VideoWriter(filePath, CV_FOURCC('M','J','P','G'), 8, cv::Size(1920, 1080));
		}

		if (!cap.read(img)) {
       		std::cout<<"Capture read error"<<std::endl;
       		break;
   		}
		   printf("is opened:show%d\n\n", img.channels());
		//    cv::cvtColor(img, img, cv::COLOR_YUV2RGB_UYVY);
		// imshow("CamShow",img);
		writer->write(img);
		// imwrite("frame.png",img);
		// if(command == "p")
		// {
		// 	string filename;
		// 	std::stringstream StrStm;
		// 	StrStm<<screenshotCnt++;
		// 	StrStm>>filename;
		// 	imwrite("img"+filename+".png",img);
		// 	command = "";
		// }
		 
		fps = cap.get(cv::CAP_PROP_FPS);
		cout<<"FPS:"<<fps<<"cnt:"<<cnt++<<endl;
		

		waitKey(1);
	}

	cap.release();
	writer->release();
	
	return 0;
}

