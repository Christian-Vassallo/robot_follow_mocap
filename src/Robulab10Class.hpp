
// Class TurtlebotMotion to control the robot motion
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define BUFLEN 2048
#define SERVICE_PORT	10000	/* hard-coded port number */

class Robulab10
{
    private:
        struct sockaddr_in myaddr, remaddr;
        socklen_t slen;

        int fd, i, timenow, recvlen;
        double lin_vel, ang_vel;
        std::string message;	        /* message buffer */
        std::ostringstream commandvelocity;
        char *string;
        char buf[2048];

    public:
        Robulab10();
        ~Robulab10();
        void move_robot(double, double);
        void stop_robot();
        int establish_connection();
};

Robulab10::Robulab10(){
        lin_vel = 0;
        ang_vel = 0;
        slen = sizeof(remaddr);
}

Robulab10::~Robulab10(){}

int Robulab10::establish_connection(){

    // Set Server address here
    char *server = "192.168.1.2";

    //Check Socket
	if ((fd=socket(AF_INET, SOCK_DGRAM, 0))==-1)
		std::cout << "socket created\n" << std::endl;

	// Bind it to all local addresses and pick any port number
	memset((char *)&myaddr, 0, sizeof(myaddr));
	myaddr.sin_family = AF_INET;
	myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myaddr.sin_port = htons(0);
	if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
		perror("bind failed");
		return 0;
	}

	// now define remaddr, the address to whom we want to send messages
	// For convenience, the host address is expressed as a numeric IP address
	// that we will convert to a binary format via inet_aton
	memset((char *) &remaddr, 0, sizeof(remaddr));
	remaddr.sin_family = AF_INET;
	remaddr.sin_port = htons(SERVICE_PORT);
	if (inet_pton(AF_INET, server, &remaddr.sin_addr)==0) {
		fprintf(stderr, "inet_aton() failed\n");
		return -1;
	}

    // if everything is ok: connection done
	std::cout << " Connection established" << std::endl;
	return 0;
}

void Robulab10::move_robot(double lin_vel, double ang_vel){

    /// Here the commands to move the robot will be sent to move it
    /// Alive: needed to avoid that WatchDog is the motion
    /// StartRobot: needed to activate the robot
    /// Drive: move command - Linear and Angular Velocity (50 means 0.5 raidant/s or 0.5 m/s)

    commandvelocity.str("");
    commandvelocity.clear();
    commandvelocity << lin_vel*100 << " " << ang_vel*100 << std::endl;

    message = "[0] Alive";
    sendto(fd, message.c_str() , message.length(), 0, (struct sockaddr *)&remaddr, slen);

    message = "[1] StartRobot";
    sendto(fd, message.c_str() , message.length(), 0, (struct sockaddr *)&remaddr, slen);
    printf("Sending message %s\n", message.c_str());

    message = "[2] Drive " + commandvelocity.str();
    sendto(fd, message.c_str() , message.length(), 0, (struct sockaddr *)&remaddr, slen);
    printf("Sending message %s\n", message.c_str());


}

void Robulab10::stop_robot(){

    /// Here the commands to stop the robot
    commandvelocity.str("");
    commandvelocity.clear();

    message = "[0] Alive";
    sendto(fd, message.c_str() , message.length(), 0, (struct sockaddr *)&remaddr, slen);

    message = "[1] StartRobot";
    sendto(fd, message.c_str() , message.length(), 0, (struct sockaddr *)&remaddr, slen);
    printf("Sending message %s\n", message.c_str());

    message = "[2] Drive 0 0";
    sendto(fd, message.c_str() , message.length(), 0, (struct sockaddr *)&remaddr, slen);
    printf("Sending message %s\n", message.c_str());

}



