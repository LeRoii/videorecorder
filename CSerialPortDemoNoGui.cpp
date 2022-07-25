#include <iostream>
#include <fstream>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <unistd.h>

#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"

#ifdef _WIN32
#include <windows.h>
#define imsleep(microsecond) Sleep(microsecond) // ms
#else
#include <unistd.h>
#define imsleep(microsecond) usleep(1000 * microsecond) // ms
#endif

#include <vector>
using namespace itas109;
using namespace std;

int countRead = 0;
int writeCnt = 0;


std::string filePath;
std::ofstream out;

class mySlot : public has_slots<>
{
public:
    mySlot(CSerialPort *sp)
    {
        recLen = -1;
        p_sp = sp;
    };

    void OnSendMessage()
    {
        // read
        recLen = p_sp->readAllData(str);
        if (recLen > 0)
        {
            countRead++;
            printf("recLen:%d\n", recLen);
            // std::ofstream outt;
            // outt.open("./filePath.txt", std::ofstream::out | std::ofstream::app);
            // if(outt.is_open())
            // {
            //     printf("filePath open!\n");
            //     outt<<"1111"<<std::endl;
            // }

            // writefile(recLen, str);
            // out.open(filePath, std::ios::out );
            // char* buffer= "asd";
            // out.write(buffer, 3);
            // if(!out.is_open())
            // {
            //     printf("not open!\n");
                
            // }
            // out<<"1";
            // out.flush();

            for(int i=0;i<recLen;i++)
            {
                printf( "  0x%02X\n", str[ i ]);
                // printf("open:%d\n",out.is_open());
                // out<<"1";
                out<<std::hex<< short(str[i])<<" ";
                writeCnt++;
                if(writeCnt > 100)
                {
                    out << std::endl;
                    writeCnt = 0;
                }
            }
            out.flush();

            // str[recLen] = '\0';
            // std::cout << "receive data : " << str << ", receive size : " << recLen << ", receive count : " << countRead << std::endl;

            // if (countRead > 7)
            // {
            //     std::cout << "close serial port when receive count > 7" << std::endl;
            //     p_sp->close();
            // }
            // else
            // {
            //     // return receive data
            //     p_sp->writeData(str, recLen);
            // }
        }
    };

private:
    mySlot(){};

private:
    CSerialPort *p_sp;

    char str[1024];
    int recLen;
   
    
};

int main()
{
    size_t index = -1;
    std::string portName;
    vector<SerialPortInfo> m_availablePortsList;
    CSerialPort sp;

    std::cout << "Version : " << sp.getVersion() << std::endl << std::endl;

    mySlot receive(&sp);

    m_availablePortsList = CSerialPortInfo::availablePortInfos();

    std::cout << "availableFriendlyPorts : " << std::endl;

    for (size_t i = 0; i < m_availablePortsList.size(); i++)
    {
        std::cout << i << " - " << m_availablePortsList[i].portName << " " << m_availablePortsList[i].description << std::endl;
    }

    if (m_availablePortsList.size() == 0)
    {
        std::cout << "No valid port" << std::endl;
    }
    else
    {
        index = 1;

        portName = m_availablePortsList[index].portName;
        std::cout << "select port name : " << portName << std::endl;

        sp.init("/dev/ttyTHS1", // windows:COM1 Linux:/dev/ttyS0
                itas109::BaudRate460800, 
                itas109::ParityNone, 
                itas109::DataBits8, 
                itas109::StopOne
                );

        sp.open();

        if (sp.isOpened())
        {
            std::cout << "open "  << " success" << std::endl;
        }
        else
        {
            std::cout << "open "  << " failed" << std::endl;
        }

        //     }
    // }

        std::time_t tt = std::chrono::system_clock::to_time_t (std::chrono::system_clock::now());
        std::stringstream ss;
        ss << std::put_time(std::localtime(&tt), "%F-%H-%M-%S");
        std::string str = "/home/nx/savedvideo/"+ss.str()+".txt";
        ss.str("");
        ss << str;
        ss >> filePath;

        
        out.open(filePath, std::ios::out );
        // if(out.is_open())
        // {
        //     printf("open!\n");
        //     out<<"1111"<<std::endl;
        // }


        // connect for read
        sp.readReady.connect(&receive, &mySlot::OnSendMessage);

        // write
        // sp.writeData("itas109", 7);

        for (;;)
        {
            imsleep(1);
        }
    }

    return 0;
}