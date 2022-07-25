#include"test.pb.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

uint64_t createtimestamp()
{
	auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    uint64_t* ptr=reinterpret_cast<uint64_t*>(&duration);
	uint64_t  tmp=*ptr;
	return tmp;
}


void printts(uint64_t ts)
{
	typedef std::chrono::duration<int, std::ratio<86400>> Days;
	std::chrono::system_clock::duration* dur_ptr=reinterpret_cast<std::chrono::system_clock::duration*>(&ts);
	auto days = std::chrono::duration_cast<Days>(*dur_ptr);
	*dur_ptr -= days;
	auto hours = std::chrono::duration_cast<std::chrono::hours>(*dur_ptr);
	*dur_ptr -= hours;
	auto minutes = std::chrono::duration_cast<std::chrono::minutes>(*dur_ptr);
	*dur_ptr -= minutes;
	auto seconds = std::chrono::duration_cast<std::chrono::seconds>(*dur_ptr);
	*dur_ptr -= seconds;
	auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(*dur_ptr);
	*dur_ptr -= milliseconds;
	auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(*dur_ptr);
	*dur_ptr -= microseconds;
	auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(*dur_ptr);
    	
    std::cout<<hours.count()+8<<":"
    		<<minutes.count()<<":"
    		<<seconds.count()<<":"
    		<<milliseconds.count()<<":"
    		<<microseconds.count()<<":"
    		<<nanoseconds.count()<<std::endl;
}



int main(int argc, char* argv[])
{
	// GOOGLE_PROTOBUF_VERIFY_VERSION;
	// if(argc<2){
	// 	std::cout<<"argc is too small"<<std::endl;
	// 	return 0;
	// }

	// std::fstream output(argv[1], std::ios::out | std::ios::binary);
	// if (!output)
	// {
	// 	std::cout<<"output file : "<< argv[1] << " is not found."<<std::endl;
	// }
	
	pb::serial_msg* pmsg;
	pb::msg_bag bag;
	
	// for(int i=0; i < 200*60*60 ; ++i)
	// {
	// 	pmsg = bag.add_msgs();
		
	// 	pmsg->set_timestamp(createtimestamp());
	// 	pmsg->set_type(pb::serial_msg_Msg_type_GPS);
	// 	pmsg->set_len(90);
	// 	char str[90]={i%256};
	// 	pmsg->set_body(str,90);
	// }
	
	// auto str=bag.SerializeAsString();
	// uint64_t len=str.size();
	// char* start=reinterpret_cast<char*>(&len);
	// output.write(start, 8);
	// output.write(str.c_str(), str.length());
	// output.close();
	
	std::fstream input(argv[1], std::ios::in | std::ios::binary);
	if (!input)
	{
		std::cout<<"input file : "<< argv[1] << " is not found."<<std::endl;
	}

	size_t read_size;
	std::string read_input;

	if(input.is_open())
	{
		input.read(reinterpret_cast<char*>(&read_size),sizeof(read_size));
		read_input.resize(read_size);
		input.read(&read_input[0], read_size);
		input.close();
	}
	
	bag.ParseFromString(read_input);
	
	int size = bag.msgs_size();
	
	for(int i=0; i< size; ++i)
	{
		pb::serial_msg msg=bag.msgs(i);
		auto timestamp=msg.timestamp();
		auto type=msg.type();
		auto len=msg.len();
		auto body=msg.body();
		
		printts(timestamp);
		std::cout<<"msg.type is "<<type<<", msg.len is "<<len<<".\n";

		for(int i=0; i<len; ++i)
		        printf("%#x ", body[i]);
		    printf("\n");

		
	
	
	}
	
	input.close();
	
	google::protobuf::ShutdownProtobufLibrary();
	return 0;
}
	
	
