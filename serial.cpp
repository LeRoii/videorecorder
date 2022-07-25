#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <termios.h>
#include <string.h>
#include <assert.h>
#include <string>
#include <vector>



// linux/include/uapi/asm-generic/termbits.h
struct termios2
{
    tcflag_t c_iflag; /* input mode flags */
    tcflag_t c_oflag; /* output mode flags */
    tcflag_t c_cflag; /* control mode flags */
    tcflag_t c_lflag; /* local mode flags */
    cc_t c_line;      /* line discipline */
    cc_t c_cc[19];    /* control characters */
    speed_t c_ispeed; /* input speed */
    speed_t c_ospeed; /* output speed */
};

#ifndef BOTHER
#define BOTHER 0010000
#endif

// linux/include/uapi/asm-generic/ioctls.h
#ifndef TCGETS2
#define TCGETS2 _IOR('T', 0x2A, struct termios2)
#endif

#ifndef TCSETS2
#define TCSETS2 _IOW('T', 0x2B, struct termios2)
#endif

static void set_baudrate (struct termios *opt, unsigned int baudrate)
{	

	cfsetispeed(opt, baudrate);
	cfsetospeed(opt, baudrate);
}

static void set_data_bit (struct termios *opt, unsigned int databit)
{
    opt->c_cflag &= ~CSIZE;
    switch (databit) {
    case 8:
        opt->c_cflag |= CS8;
		printf("8\n");
        break;
    case 7:
        opt->c_cflag |= CS7;
		printf("7\n");
        break;
    case 6:
        opt->c_cflag |= CS6;
        break;
    case 5:
        opt->c_cflag |= CS5;
        break;
    default:
        opt->c_cflag |= CS8;
        break;
    }
}

static void set_parity (struct termios* const opt, const char parity)
{
    switch (parity) {
    case 'E':                  /* even */
    case 'e':
        //opt->c_cflag |= PARENB;
        //opt->c_cflag &= ~PARODD;
        opt->c_cflag |= PARENB;  // PARENB：产生奇偶位，执行奇偶校验
        opt->c_cflag &= ~PARODD; // PARODD：若设置则为奇校验,否则为偶校验
        opt->c_cflag |= INPCK;   // INPCK：使奇偶校验起作用
        opt->c_cflag |= ISTRIP;  // ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
        break;
    case 'O':                  /* odd */
    case 'o':
        //opt->c_cflag |= PARENB;
        //opt->c_cflag |= ~PARODD;
		opt->c_cflag |= PARENB; // PARENB：产生奇偶位，执行奇偶校验
        opt->c_cflag |= PARODD; // PARODD：若设置则为奇校验,否则为偶校验
        opt->c_cflag |= INPCK;  // INPCK：使奇偶校验起作用
        opt->c_cflag |= ISTRIP; // ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
        break;
	case 'N':                  /* no parity check */
    case 'n':
    default:                   /* no parity check */
        opt->c_cflag &= ~PARENB; // PARENB：产生奇偶位，执行奇偶校验
		opt->c_cflag &= ~PARODD;
        opt->c_cflag &= ~INPCK;  // INPCK：使奇偶校验起作用
        break;
    }
}

static void set_stopbit (struct termios *opt, const char *stopbit)
{
    if (0 == strcmp (stopbit, "1")) {
        opt->c_cflag &= ~CSTOPB; /* 1 stop bit */
    }	else if (0 == strcmp (stopbit, "1.5")) {
        opt->c_cflag &= ~CSTOPB; /* 1.5 stop bit */
    }   else if (0 == strcmp (stopbit, "2")) {
        opt->c_cflag |= CSTOPB;  /* 2 stop bits */
    } else {
        opt->c_cflag &= ~CSTOPB; /* 1 stop bit */
    }
}

int  set_port_attr (
              int fd,
              int  baudrate,          // B1200 B2400 B4800 B9600 .. B115200 B230400
              int  databit,           // 5, 6, 7, 8
              const char *stopbit,    //  "1", "1.5", "2"
              char parity,            // N(o), O(dd), E(ven)
              int vtime,
              int vmin )
{
	struct termios opt;
	tcgetattr(fd, &opt);
	//设置波特率
	set_baudrate(&opt, baudrate);
	
	opt.c_cflag &= ~CMSPAR;
	opt.c_cflag &= ~HUPCL;
	//设置校验位
	set_parity(&opt, parity);
	//设置数据位
	set_data_bit(&opt, databit);

	//设置停止位
	set_stopbit(&opt, stopbit);

	opt.c_cflag 		 |= CLOCAL | CREAD ;      /* | CRTSCTS */

	printf("cflag is %#x\n", opt.c_cflag);
	//其它设置
	//opt.c_oflag 		 = 0;
	//opt.c_lflag            	|= 0;

    //opt.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO | ECHOE | ECHOK| ECHONL | NOFLSH | XCASE | TOSTOP | ECHOPRT | ECHOCTL | ECHOKE | ECHOKE | FLUSHO | EXTPROC);
	//opt.c_iflag |= IGNBRK;
	//opt.c_iflag &= ~(BRKINT | IGNPAR | PARMRK | IGNCR | ICRNL | INLCR | INPCK | ISTRIP | IXON | IXOFF | IXANY | IUCLC | IMAXBEL | IUTF8);
	//opt.c_oflag &= ~(OPOST | OLCUC | OCRNL | ONLCR | ONOCR | ONLRET | OFILL | OFDEL);
	//opt.c_oflag |= NL0 | CR0 | TAB0 | BS0 | VT0 | FF0;
	// 设置输出模式为原始输出
    opt.c_oflag &= ~OPOST; // OPOST：若设置则按定义的输出处理，否则所有c_oflag失效

    //设置本地模式为原始模式
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    /*
     *ICANON：允许规范模式进行输入处理
     *ECHO：允许输入字符的本地回显
     *ECHOE：在接收EPASE时执行Backspace,Space,Backspace组合
     *ISIG：允许信号
     */

    opt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    /*
     *BRKINT：如果设置了IGNBRK，BREAK键输入将被忽略
     *ICRNL：将输入的回车转化成换行（如果IGNCR未设置的情况下）(0x0d => 0x0a)
     *INPCK：允许输入奇偶校验
     *ISTRIP：去除字符的第8个比特
     *IXON：允许输出时对XON/XOFF流进行控制 (0x11 0x13)
     */

    // 设置等待时间和最小接受字符
    //options.c_cc[VTIME] = 0; // 可以在select中设置
    //options.c_cc[VMIN] = 1;  // 最少读取一个字符
	opt.c_cc[VTIME]     	 = vtime;
	opt.c_cc[VMIN]         	 = vmin;
	
	tcflush (fd, TCIOFLUSH);
	return (tcsetattr (fd, TCSANOW, &opt));
}

int  init_serial(const char* serial_name,
				int baudrate,
				int databit,
				const char* stopbit,
				char parity)
{
	int fd = open(serial_name, O_RDWR | O_NOCTTY);
	if(fd<0)
		return fd;

    if(fcntl(fd, F_SETFL, 0)<0)
        printf("block set failed\n");
    else
        printf("block set pass\n");
	int ret = set_port_attr(fd, baudrate, databit, stopbit, parity, 1, 0);
	if(ret<0)
        return ret;
    return fd;
}

int uart_read(int fd, unsigned char * buf, unsigned int len, unsigned int wait_usec)
{
    struct timeval tv;
    fd_set set;
	struct timeval* ptv=NULL;

    FD_ZERO(&set);
    FD_SET(fd, &set);
	if(wait_usec)
	{
		memset(&tv, 0, sizeof(tv));
		tv.tv_sec=0;
		tv.tv_usec=wait_usec;
		ptv=&tv;		
	}
	

    if(select(fd+1, &set, NULL, NULL, ptv)>0)
    {
        if(FD_ISSET(fd, &set))
        {
            int ret=read(fd, buf, len);
            return ret;            
        }
    }
    return -1;
}
/*
uint64_t createtimestamp()
{
	auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    uint64_t* ptr=reinterpret_cast<uint64_t*>(&duration);
	uint64_t  tmp=*ptr;
	return tmp;
}
*/



// int main(int argc, char* argv[])
// {
//     int fd=init_serial("/dev/ttyTHS1", B921600, 8, "1", 'n');
//     //int fd=init_serial("/dev/ttyTHS0", B460800, 8, "1", 'e');
//     assert(fd>=0);
//     unsigned long sum=0, gsum=0, csum=0, wsum=0;	
// 	int maxSize=512;
// 	std::string str;
// 	str.resize(512);
// 	std::vector<int> warray;
//     while(1)
//     {         
// 		ioctl(fd, FIONREAD, &maxSize);
// 		if(maxSize>str.size())
// 			str.resize(maxSize);
//         int ret=uart_read(fd, reinterpret_cast<unsigned char*>(&str[0]), maxSize, 0);
// 		if(ret>0){
// 		    printf("rcv No.%d, len %d: ", sum++, ret);
// 		    for(int i=0; i<ret; ++i)
// 		        printf("%#x ", str[i]);
// 		    printf("\n");
// 			if((ret==140)&&(str[0]==0x55)&&(str[1]=0xaa)) ++csum;
// 			else
// 			{
// 					++wsum;
// 					warray.push_back(ret);
					

// 			}
// 			gsum+=onRcvGPSMsg(reinterpret_cast<unsigned char*>(&str[0]), ret);
			
// 			printf("csum %d, gsum %d, wsum %d:", csum, gsum, wsum);
// 			if(!warray.empty())
// 			{
// 				for(auto & i:warray)
// 					printf(" %d",i);
// 			}
// 			printf(".\n");
				
// 		}
//     }
//     return 0;

// }
