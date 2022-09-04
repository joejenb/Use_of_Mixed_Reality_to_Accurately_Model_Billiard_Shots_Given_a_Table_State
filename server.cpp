#define _WIN32_WINNT 0x501
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <math.h>


// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")


#define DEFAULT_PORT 8000
#define PI 3.14159265

 
struct Shot_Data
{
	float resultant_x1;
    float resultant_y1;
    float resultant_x2;
    float resultant_y2;
    float object_x1;
    float object_y1;
    float object_x2;
    float object_y2;
    float cue_x1;
    float cue_y1;
    float cue_x2;
    float cue_y2;
    float radi;
};

/* bool operator==( const Shot_Data& a1, const Shot_Data& a2 )
 {
     return (a1.white_x == a2.white_x) &&
         (a1.white_y == a2.white_y) &&
         (a1.object_x == a2.object_x) &&
         (a1.object_y == a2.object_y) &&
         (a1.radi == a2.radi) &&
         (a1.cue_x1 == a2.cue_x1) &&
         (a1.cue_y1 == a2.cue_y1) &&
         (a1.cue_x2 == a2.cue_x2) &&
         (a1.cue_y2 == a2.cue_y2);
 }*/
 
 std::string to_json(Shot_Data& shot_data )
 {
     std::string json_obj = "{ \"resultant_x1\": " + std::to_string(shot_data.resultant_x1);
     json_obj += ", \"resultant_y1\": " + std::to_string(shot_data.resultant_y1);
     json_obj += ", \"resultant_x2\": " + std::to_string(shot_data.resultant_x2);
     json_obj += ", \"resultant_y2\": " + std::to_string(shot_data.resultant_y2);
     json_obj += ", \"object_x1\": " + std::to_string(shot_data.object_x1);
     json_obj += ", \"object_y1\": " + std::to_string(shot_data.object_y1);
     json_obj += ", \"object_x2\": " + std::to_string(shot_data.object_x2);
     json_obj += ", \"object_y2\": " + std::to_string(shot_data.object_y2);
     json_obj += ", \"cue_x1\": " + std::to_string(shot_data.cue_x1);
     json_obj += ", \"cue_y1\": " + std::to_string(shot_data.cue_y1);
     json_obj += ", \"cue_x2\": " + std::to_string(shot_data.cue_x2);
     json_obj += ", \"cue_y2\": " + std::to_string(shot_data.cue_y2);
     json_obj += ", \"radi\": " + std::to_string(shot_data.radi) + "}";
     return json_obj;
 }
 
 
int main()
{
    WSAData wsaData;
    SOCKADDR_IN addr;
    
    addr.sin_family = AF_INET;
    addr.sin_port = htons(DEFAULT_PORT);
    addr.sin_addr.s_addr = inet_addr("192.168.1.164");

    int angle = 0;
	Shot_Data shot_data = { 0, 0.25, 0, 0.35, 0, -0.25, 0, -0.35, 0, 0.35, 0, -0.35, 0.08 };
    while (true)
    {
		int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
		SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        result = connect(s, (SOCKADDR*)&addr, sizeof(addr));

        angle++;
        angle = angle % 360;

        shot_data.resultant_y2 = shot_data.resultant_y1 + (0.1 * sin(angle * PI / 180));
        shot_data.resultant_x2 = shot_data.resultant_x1 + (0.1 * cos(angle * PI / 180));
        shot_data.object_y2 = shot_data.object_y1 - (0.1 * sin(angle * PI / 180));
        shot_data.object_x2 = shot_data.object_x1 - (0.1 * cos(angle * PI / 180));
        shot_data.cue_y2 = shot_data.object_y2;
        shot_data.cue_x2 = shot_data.object_x2;
        shot_data.cue_y1 = shot_data.resultant_y2;
        shot_data.cue_x1 = shot_data.resultant_x2;

        int count = 0;
        char sendBuf[1024];
        strcpy(sendBuf, to_json(shot_data).c_str());
        
        printf("\n%d  %d\n", angle, strlen(sendBuf));
        printf(sendBuf);
        int sendResult;
        sendResult = send(s, sendBuf, strlen(sendBuf), 0);
        shutdown(s, SD_SEND);
        Sleep(0.03);
    }
}
