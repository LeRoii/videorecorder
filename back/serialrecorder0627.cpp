/*
   Copyright 2016 rm5248

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
*/

#include <stdint.h>
#include <stdio.h>
#include <cserial/c_serial.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <unistd.h>

/**
 */


int main( int argc, char** argv ){
    c_serial_port_t* m_port;
    c_serial_control_lines_t m_lines;
    int status;
    int bytes_read;
     int buff_size = 200;
    uint8_t data[ 255 ];
    uint8_t buff[ buff_size ];
    int data_length;
    int write_ptr = 0;
    //int ptr = 0;
    int lenth = 0;
    int index = 0;
    int write_num = 0;
    bool first = true;
    int x;

    /*
     * Use the first argument as the port to open
     */
    if( argc != 2 ){
        fprintf( stderr, "ERROR: First argument must be serial port\n" );
		/*
		 * Since no port was provided, print the available ports
		 */
		const char** port_list = c_serial_get_serial_ports_list();
		x = 0;
		printf("Available ports:\n");
		while( port_list[ x ] != NULL ){
			printf( "%s\n", port_list[ x ] );
			x++;
		}
		c_serial_free_serial_ports_list( port_list );
        return 1;
    }
 
    /*
     * Set the global log function.
     * We will use a simple log function that prints to stderr
     */
    c_serial_set_global_log_function( c_serial_stderr_log_function );

    /*
     * Allocate the serial port struct.
     * This defaults to 9600-8-N-1
     */
    if( c_serial_new( &m_port, NULL ) < 0 ){
        fprintf( stderr, "ERROR: Unable to create new serial port\n" );
        return 1;
    }

    /*
     * The port name is the device to open(/dev/ttyS0 on Linux,
     * COM1 on Windows)
     */
    if( c_serial_set_port_name( m_port, argv[ 1 ] ) < 0 ){
        fprintf( stderr, "ERROR: can't set port name\n" );
    }

    /*
     * Set various serial port settings.  These are the default.
     */
    // c_serial_set_baud_rate( m_port, CSERIAL_BAUD_115200 );
     c_serial_set_baud_rate( m_port, CSERIAL_BAUD_115200 );
    c_serial_set_data_bits( m_port, CSERIAL_BITS_8 );
    c_serial_set_stop_bits( m_port, CSERIAL_STOP_BITS_1 );
    c_serial_set_parity( m_port, CSERIAL_PARITY_EVEN );
    c_serial_set_flow_control( m_port, CSERIAL_FLOW_NONE );

    printf( "Baud rate is %d\n", c_serial_get_baud_rate( m_port ) );

    /*
     * We want to get all line flags when they change
     */
    c_serial_set_serial_line_change_flags( m_port, CSERIAL_LINE_FLAG_ALL );

    status = c_serial_open( m_port );
    if( status < 0 ){
        fprintf( stderr, "ERROR: Can't open serial port\n" );
        return 1;
    }

    uint8_t buf[6] = {0x81, 0x01, 0x4, 0x7, 0x35, 0xff};
    data_length = 6;

    // while(1)
    // {
    //     status = c_serial_write_data( m_port, buf,  &data_length );
    //     if( status < 0 ){
    //         printf("send failed\n");
    //     }
    // }

    std::string filePath;
    std::time_t tt = std::chrono::system_clock::to_time_t (std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&tt), "%F-%H-%M-%S");
    std::string str = "/home/nxsd/savedvideo/"+ss.str()+".txt";
    ss.str("");
    ss << str;
    ss >> filePath;

    std::ofstream out;
    out.open(filePath, std::ios::out);
    if(out.is_open())
    {
        printf("open!\n");
    }

    /*
     * Listen for anything that comes across, and echo it back.
     */
    do{
        data_length = 255;
        status = c_serial_read_data( m_port, data, &data_length, &m_lines );
       // printf("status: %d m_lines: %s\n",status,m_lines);
        if( status < 0 ){
            break;
        }

       // memset(buff,0,buff_size);
        
        printf( "Got %d bytes of data\n", data_length );
        for( x = 0; x < data_length; x++ ){
            printf( "    0x%02X (ASCII: %c)\n", data[ x ], data[ x ] );
            if(data[x] == 0xCC && data[x+1] == 0x55)
            {
                //index = x;
                //write = !write;
                lenth = write_num;
                write_num = 0;

            if (first)
            {
                first = false;
                index = x;
            }
            else
            {
                for (int i = index; i < lenth; i++)
                {
                if (i == buff_size)
                {
                    i=0;
                }
                out<<std::hex<< short(buff[i])<<" ";
                index = x;
                }

                out<<std::endl; 
            }


            }
            buff[write_ptr++] = data[x];
              
            write_num++;
             printf( " write_ptr:%d  --- 0x%02X x:%d\n",write_ptr, buff[ x ],x );
            
            if (write_ptr == buff_size)
            {
                write_ptr = 0;
            }

            
            
        }
        printf( "Serial line state: CD: %d CTS: %d DSR: %d DTR: %d RTS: %d RI: %d\n",
            m_lines.cd, 
            m_lines.cts,
            m_lines.dsr,
            m_lines.dtr,
            m_lines.rts,
            m_lines.ri );


    }while( 1 );
}
