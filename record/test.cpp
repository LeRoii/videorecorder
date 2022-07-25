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



int main(int argc, char* argv[])
{
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	if(argc<2){
		std::cout<<"argc is too small"<<std::endl;
		return 0;
	}

	std::fstream output(argv[1], std::ios::out | std::ios::binary);
	if (!output)
	{
		std::cout<<"output file : "<< argv[1] << " is not found."<<std::endl;
	}
	
	pb::serial_msg* pmsg;
	pb::msg_bag bag;
	
	for(int i=0; i < 200*60*60 ; ++i)
	{
		pmsg = bag.add_msgs();
		
		pmsg->set_timestamp(createtimestamp());
		pmsg->set_type(pb::serial_msg_Msg_type_GPS);
		pmsg->set_len(90);
		char str[90]={i%256};
		pmsg->set_body(str,90);
	}
	
	auto str=bag.SerializeAsString();
	uint64_t len=str.size();
	char* start=reinterpret_cast<char*>(&len);
	output.write(start, 8);
	output.write(str.c_str(), str.length());
	output.close();
	
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
		std::cout<<"msg.type is "<<type<<", msg.len is "<<len<<".\n";
	
	
	}
	
	input.close();
	
	google::protobuf::ShutdownProtobufLibrary();
	return 0;
}
	
	
