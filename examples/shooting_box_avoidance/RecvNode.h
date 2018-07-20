#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <memory>
#include <thread>
#include <iostream>
#include <functional>

#define BUFSIZE 2048

class RecvNode {
public:
  RecvNode(unsigned int port){
    service_port_ = port;
    addrlen_ = sizeof(remaddr_);
  }
  ~RecvNode(){
    should_run_ = false;
    thread_->join();
    thread_.reset();
    close(fd_);
  }

  bool InitServer(){
    /* create a UDP socket */
    if ((fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      perror("cannot create socket\n");
      return false;
    }

    /* bind the socket to any valid IP address and a specific port */
    memset((char *)&myaddr_, 0, sizeof(myaddr_));
    myaddr_.sin_family = AF_INET;
    myaddr_.sin_addr.s_addr = htonl(INADDR_ANY);
    myaddr_.sin_port = htons(service_port_);

    if (bind(fd_, (struct sockaddr *)&myaddr_, sizeof(myaddr_)) < 0) {
      perror("bind failed");
      return false;
    }

    // open a thread and listen to port
    should_run_ = true;
    thread_ = std::make_shared<std::thread>(&RecvNode::ListenToPort,this);
    printf("waiting on port %d\n", service_port_);

    return true;
  }

  void RegisterPoseUpdateCallBack(const std::function<void(float,float,float)>& fcn){
    pose_callback_ = fcn;
  }

  void RegisterObsUpdateCallBack(const std::function<void(float,float)>& fcn){
    obs_callback_ = fcn;
  }

private:
	struct sockaddr_in myaddr_;	/* our address */
	struct sockaddr_in remaddr_;	/* remote address */
	socklen_t addrlen_;		/* length of addresses */
	int fd_;				/* our socket */
	unsigned int service_port_;

	std::shared_ptr<std::thread> thread_;
	bool should_run_;

	std::function<void(float,float,float)> pose_callback_;
	std::function<void(float,float)> obs_callback_;

	#pragma pack(1)
	struct PoseMsg {
		PoseMsg(){
			x = 0;
			y = 0;
			yaw = 0;
		}
		const unsigned char id = 1;
		float x;
		float y;
		float yaw;
	};
	#pragma pack()

	#pragma pack(1)
	struct ObsMsg {
		ObsMsg(){
			x = 0;
			y = 0;
		}
		const unsigned char id = 2;
		float x;
		float y;
	};
	#pragma pack()

	void ListenToPort(){
		int recvlen;
		unsigned char buf[BUFSIZE];
		while(should_run_) {
			recvlen = recvfrom(fd_, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr_, &addrlen_);
//			printf("received %d bytes\n", recvlen);
			if (recvlen > 0) {
//				std::cout << "first char is " << (int)buf[0] << std::endl;
				switch ((int)buf[0]) {
					case 1:
					{
						PoseMsg msg;
						memcpy(&msg,&buf,sizeof(PoseMsg));
						if(pose_callback_){
							pose_callback_(msg.x,msg.y,msg.yaw);
						}
						break;
					}
					case 2:
					{
						ObsMsg msg;
						memcpy(&msg,&buf,sizeof(ObsMsg));
						if(obs_callback_){
							obs_callback_(msg.x,msg.y);
						}
						break;
					}
					default:
						std::cerr << "OOPS!! Received wrong Header." << std::endl;
						break;
				}
			} else {
				std::cerr << "OOPS!! Packet was of size 0 " << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}

};
