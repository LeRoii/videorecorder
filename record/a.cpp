#include <chrono>
#include <iostream>   
using namespace std;
using namespace chrono;

int main(int argc, char* argv[])
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    
    typedef std::chrono::duration<int, std::ratio<86400>> Days;
    
    auto days = std::chrono::duration_cast<Days>(duration);
	duration -= days;
	auto hours = std::chrono::duration_cast<std::chrono::hours>(duration);
	duration -= hours;
	auto minutes = std::chrono::duration_cast<std::chrono::minutes>(duration);
	duration -= minutes;
	auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
	duration -= seconds;
	auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
	duration -= milliseconds;
	auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration);
	duration -= microseconds;
	auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);
    	
    std::cout<<hours.count()+8<<":"
    		<<minutes.count()<<":"
    		<<seconds.count()<<":"
    		<<milliseconds.count()<<":"
    		<<microseconds.count()<<":"
    		<<nanoseconds.count()<<std::endl;
	std::cout<<sizeof(duration)<<std::endl;
	
	duration = now.time_since_epoch();
	uint64_t* ptr=reinterpret_cast<uint64_t*>(&duration);
	uint64_t  tmp=*ptr;
	std::chrono::system_clock::duration* dur_ptr=reinterpret_cast<std::chrono::system_clock::duration*>(&tmp);
	days = std::chrono::duration_cast<Days>(*dur_ptr);
	*dur_ptr -= days;
	hours = std::chrono::duration_cast<std::chrono::hours>(*dur_ptr);
	*dur_ptr -= hours;
	minutes = std::chrono::duration_cast<std::chrono::minutes>(*dur_ptr);
	*dur_ptr -= minutes;
	seconds = std::chrono::duration_cast<std::chrono::seconds>(*dur_ptr);
	*dur_ptr -= seconds;
	milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(*dur_ptr);
	*dur_ptr -= milliseconds;
	microseconds = std::chrono::duration_cast<std::chrono::microseconds>(*dur_ptr);
	*dur_ptr -= microseconds;
	nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(*dur_ptr);
    	
    std::cout<<hours.count()+8<<":"
    		<<minutes.count()<<":"
    		<<seconds.count()<<":"
    		<<milliseconds.count()<<":"
    		<<microseconds.count()<<":"
    		<<nanoseconds.count()<<std::endl;
	
    return 0;
}
