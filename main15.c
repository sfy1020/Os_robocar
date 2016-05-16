

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "uart_api.h"
#include "robot_protocol.h"
#include "keyboard.h"

typedef struct node{
	int f, s, t; 
	char dir; 

	int w;
}node;

int num  = 14; // node 갯수


int *pre; // 이전 노드
int *weight; // 가중치

int **adjList; // 인접행렬
int *visit; // 방문여부

int pathcnt = 0;//패스 위치를 확인할 인덱스
char *temparr;//회전시 패스를 정리하기위한 배열
int *temppath;
int *path; // 현재 진행방향
int *before; // 진행방향 이전 노드
int *after; // 진행방향 다음 노드
char *direct; // 노드마다 결정해줄 방향

unsigned char Line_Value, tmp_line = 0xff;

//{시작 다음, 그다음 , 방향, 가중치}
node road[] = {
	{0, 1, 2, 'L', 5},
	{1, 2, 3, 'R', 4},
	{1, 2, 4, 'L', 4},
	{2, 3, 6, 'L', 4},
	{2, 4, 8, 'L', 4},
	{2, 1, 0, 'R', 1},
	{3, 6, 5, 'L', 4},
	{3, 2, 4, 'R', 4},
	{3, 2, 1, 'L', 5},
	{3, 6, 12, 'F', 4},
	{3,6,7,'L',3},
	{4, 8, 7, 'R', 4},
	{4,2,1,'L',5},
	{4, 2, 3, 'F', 3},
	{5, 6, 7, 'F', 3},
	{5, 6, 3, 'L', 4},
	{5,6,12,'R',4},
	{6, 12, 13, 'R', 3},
	{6, 12, 11, 'L', 5},
	{6,5,13,'L',4},
	{6, 3, 2, 'R', 3},
	{6, 7, 9, 'R', 2},
	{6, 7, 8, 'F', 4},
	{7,9,10,'L',2},
	{7,6,12,'L',4},
	{7, 6, 3, 'R', 4},
	{7, 8, 4, 'L', 4},
	{7, 6, 5, 'F', 3},//-----------------------------------
	{8, 7, 9, 'L', 2},
	{8,7,6,'F',3},
	{8,4,2,'L',4},
	{9,10,11,'R',2},
	{9,7,6,'L',3},
	{9,7,8,'R',4},
	{10,11,12,'R',5},
	{10,9,7,'R',2},
	{11,12,13,'F',3},
	{11,12,6,'R',4},
	{11,10,9,'L',2},
	{12,6,3,'F',4},
	{12,6,5,'L',3},
	{12,6,7,'R',3},
	{12,11,10,'L',2},
	{ 13, 5, 6 , 'R',4},
	{ 13,12,6,'L',3 }
};



void Dijkstra(int start, int end);
void Clear();
void LineFind();
void ClearMap();


int main()
{

	int i=0,j=0;
	pre = (int *)malloc(num * sizeof(int)); 
	weight = (int *)malloc(num * sizeof(int)); 
	direct = (char *)malloc((num+1) * sizeof(char));
	path = (int *)malloc((num + 1) * sizeof(int));
	before = (int *)malloc(num * sizeof(int));
	after = (int *)malloc(num * sizeof(int));
	temparr = (char *)malloc((num + 1) * sizeof(int));
	adjList = (int **)malloc(num * sizeof(int));
	temppath = (int *)malloc((num+1) * sizeof(int));


	//인접배열 생성 및 맵 입력
	for(i=0; i<num; i++)
		adjList[i] =  (int *)malloc(num * sizeof(int));

	for(i=0; i<num; i++) 
		for(j=0; j<num; j++)
			adjList[i][j]=0;
	
	for(i=0; i<sizeof(road)/sizeof(road[0]); i++){
			printf("%d ",i);
		if(road[i].f == 0 && road[i].s == 0 && road[i].t ==0)
			break;

		else if(adjList[(road[i].s)][(road[i].t)] == 0){
			adjList[(road[i].s)][(road[i].t)] = road[i].w;
			adjList[(road[i].t)][(road[i].s)] = road[i].w;
		}
	}

	
	Clear();
	
	Dijkstra(0,13);

	
	
	int ret;
	
	ret = user_uart1_open("SAC1");    

	if (ret < 0)
	{
		printf("\n UART1 Open error!");
		exit(0);
	}   

	user_uart1_config(115200, 8, UART_PARNONE, 1);

	init_keyboard();

	printf("**************************\n");
	printf(" RoboCAR Linetracer exam\n");
	printf("**************************\n");


	RoboCAR_AllMotor_Control(STOP,0);
	while(1)//Line tracing 부분
	{
		//키보드 입력시 종료
		if(kbhit()) break;
		
		Line_Value= RoboCAR_Get_InfraredRay_Data();//현재 라인 값을 입력받는다
	
			printf("index ->%d\n",path[pathcnt]);  //가고있는 방향

			printf("InfraredRay = 0x%x\n",Line_Value); //현재 읽은 센서값 출력
		

		if(pathcnt!=-1 &&path[pathcnt]>13)  //13번 노드(종료점) 이상 갔을시 정지
		{   
			printf("befor: %d, Path: %d", before[pathcnt],path[pathcnt]);
			printf("Finish\n");
			break;
		}
	
		switch(Line_Value){  //센서 읽어들인 값에 대한 캐이스
		case 0xe7:   // 1110 0111
		case 0xc3:   // 1100 0011  
		case 0xef:   // 1110 1111
		case 0xf7:  // 1111 0111   
			RoboCAR_AllMotor_Control(FORWARD,70);   //전진
			break;

			//라인 중앙을 맞추기 위한 좌회전
		case 0xf3: //1111 0011
		case 0xfc: //1111 1100
		case 0xfd: //1111 1101
		case 0xfe: //1111 1110
		case 0xe1: //1110 0001
			RoboCAR_LeftMotor_Control(BACKWARD,80);                   
			RoboCAR_RightMotor_Control(FORWARD, 80);
			break;
			//라인 중앙을 맞추기 위한 우회전
		case 0xCF: // 1100 1111
		case 0xDF: // 1101 1111
		case 0x9F: // 1001 1111
		case 0x3f: // 0011 1111
		case 0x3e: // 0011 1110
		case 0xbf: // 1011 1111
		case 0x7f: // 0111 1111
		case 0x1f: // 0001 1111
		case 0xc7: // 1100 0111
		case 0x1e: // 0001 1110
		case 0x47: // 0100 0111
			RoboCAR_LeftMotor_Control(FORWARD,80);
			RoboCAR_RightMotor_Control(BACKWARD,80);
			break;
		//좌회전할 노드 
		case 0xe0: //1110 0000
		case 0xf0: //1111 0000
		case 0xf8: //1111 1000
			if (direct[pathcnt] == 'L')
				//좌회전
			{

				printf("4-L------------------------\n");
				RoboCAR_AllMotor_Control(FORWARD, 80);
				usleep(250 * 1000);
				RoboCAR_LeftMotor_Control(BACKWARD, 80);
				RoboCAR_RightMotor_Control(FORWARD, 80);
				usleep(350 * 1000);
				LineFind();
				pathcnt++; //path index 증가
				break;
			}
			else if (direct[pathcnt] == 'R')//우회전
			{
				printf("4-R------------------------\n");
				usleep(250 * 1000);
				RoboCAR_LeftMotor_Control(FORWARD, 70);
				RoboCAR_RightMotor_Control(BACKWARD, 80);
				usleep(350 * 1000);
				LineFind();
				pathcnt++;
				break;
			}
			else//직진시
			{
				printf("4-F------------------------\n");
				LineFind();
				pathcnt++;
				break;

			}
			break;
		//우회전할 노드
		case 0x0e: // 0000 1110
		case 0x0f: // 0000 1111
		case 0x07:  // 0000 0111
			if (direct[pathcnt] == 'L')
				//좌회전
			{

				printf("4-L------------------------\n");
				RoboCAR_AllMotor_Control(FORWARD, 80);
				usleep(250 * 1000);
				RoboCAR_LeftMotor_Control(BACKWARD, 80);
				RoboCAR_RightMotor_Control(FORWARD, 80);
				usleep(350 * 1000);
				LineFind();
				pathcnt++; //path index 증가
				break;
			}
			else if (direct[pathcnt] == 'R')//우회전
			{
				printf("4-R------------------------\n");
				usleep(250 * 1000);
				RoboCAR_LeftMotor_Control(FORWARD, 70);
				RoboCAR_RightMotor_Control(BACKWARD, 80);
				usleep(350 * 1000);
				LineFind();
				pathcnt++;
				break;
			}
			else//직진시
			{
				printf("4-F------------------------\n");
				LineFind();
				pathcnt++;
				break;

			}
			break;
		case 0x00: //»ï°Åž®,»ç°Åž®žŠ žž³µÀ» °æ¿ì¿¡ 
			if (path[pathcnt]==6){//------------------------------------------------------------------------------------»ç°Åž®

				if (direct[pathcnt] == 'L')
					//좌회전
				{

					printf("4-L------------------------\n");
					RoboCAR_AllMotor_Control(FORWARD,80);
					usleep(250*1000);		
					RoboCAR_LeftMotor_Control(BACKWARD,80);
					RoboCAR_RightMotor_Control(FORWARD,80);
					usleep(350*1000);
					LineFind();
					pathcnt++; //path index 증가
					break;
				}
				else if (direct[pathcnt] == 'R')//우회전
				{
					printf("4-R------------------------\n");
					usleep(250*1000);					
					RoboCAR_LeftMotor_Control(FORWARD,70);
					RoboCAR_RightMotor_Control(BACKWARD,80);
					usleep(350*1000);
					LineFind();
					pathcnt++;
					break;
				}
				else//직진시
				{
					printf("4-F------------------------\n");
					LineFind();
					pathcnt++;
					break;

				}
			}
			//------------------------------------------------------------------------------------»ï°Åž®
			else{//3wayrode
				if (direct[pathcnt] == 'L')
					//좌회전
				{

					printf("4-L------------------------\n");
					RoboCAR_AllMotor_Control(FORWARD, 80);
					usleep(250 * 1000);
					RoboCAR_LeftMotor_Control(BACKWARD, 80);
					RoboCAR_RightMotor_Control(FORWARD, 80);
					usleep(350 * 1000);
					LineFind();
					pathcnt++; //path index 증가
					break;
				}
				else if (direct[pathcnt] == 'R')//우회전
				{
					printf("4-R------------------------\n");
					usleep(250 * 1000);
					RoboCAR_LeftMotor_Control(FORWARD, 70);
					RoboCAR_RightMotor_Control(BACKWARD, 80);
					usleep(350 * 1000);
					LineFind();
					pathcnt++;
					break;
				}
				else//직진시
				{
					printf("4-F------------------------\n");
					LineFind();
					pathcnt++;
					break;

				}
			}
		case 0xFF: 
			printf("path : %d , pathcnt : %d \n",path[pathcnt],pathcnt);
			//Èò»öÀ» žž³µÀ» °æ¿ì¿¡ŽÂ À¯ÅÏÀ» ÇÏ°Ô Œ³Á€! Èò»öÀ» ÀÎœÄÇÒ¶§ŽÂ À¯ÅÏÀ» ÇÑŽÙ.      
			RoboCAR_Move_Angle(LEFT_ROTATION,100,180); 
			//¿ÞÂÊÀž·Î 180µµ ÈžÀü œÃÅ²ŽÙ.
			ClearMap();
			LineFind();
			break;   
		}// end switch   
	}//end while      
	RoboCAR_AllMotor_Control(STOP,0); // RoboCar Stop
	user_uart1_close();
	close_keyboard() ;
	return 0;
}





//////////////////////////////////////////////////////////
void Dijkstra(int start, int end)
{
	int *visit = (int *)malloc(num * sizeof(int));
	int i, j, w, z, now, first, second, third;
	int index = 0, min_node = 0;
	int pathcount = 0, dircount = 0;
	int temp = end;
	int count = 1;

	for (i = 0; i<num; i++)
		visit[i] = 0;

	now = start;
	path[pathcount] = now;
	pathcount++;
	for (i = 0; i<num; i++) { // start point로 부터 연결되어 있는 Node들의 가중치를 weight배열에 추가	
		if (adjList[now][i] != 0) {
			weight[i] = adjList[now][i];
		}
	}
	for (i = 0; i<num; i++) {
		if (weight[i] != 100000 && i != start) { // weight배열에 추가된 정점들의 부모를 start로 지정	
			pre[i] = start;
		}
	}

	visit[now] = 1; // start point는 방문한 것으로 표시하여 다시 선택이 되지 않게 한다.

	while (now != end) { // end point까지 최단 거리를 찾기 위한 while 문
		for (i = 0; i<num; i++) {
			if (visit[i] == 0) { // 방문 안한 것 중에 min 값을 결정(초기)
				min_node = i;
				break;
			}
		}

		for (j = 0; j<num; j++) { // 위의 for문에서 정한 min 값을 기준으로 가장 작은 가중치를 가지고 있는 정점 추출		
			if (weight[min_node] >= weight[j] && visit[j] != 1) {
				min_node = j;
			}
		}

		/** 추출한 min_index를 now에 넣고 visit배열에 방문하였다고 표시 **/
		now = min_node;
		visit[min_node] = 1;
		//path[pathcount] = now;
		//pathcount++;

		for (w = 0; w<num; w++) {// now로 부터 연결된 정점들의 가중치를 수정
			if (adjList[now][w]>0) {
				if (visit[w] != 1) {
					if (weight[w] > weight[now] + adjList[now][w]) { // 현재 가지고 있는 가중치보다 지금 경우가 더 작을 경우 수정					
						weight[w] = weight[now] + adjList[now][w];
						pre[w] = now;
					}
				}
			}
		}
	}


	while (start != pre[temp]) {
		temp = pre[temp];

		count++;
	}
	/*
	for (i = 0; i < num; i++) {
	printf("node : %d  ,", path[i]);
	}
	printf("\n");
	*/

	pathcount = 0;
	//count = pathcount;
	temp = end;
	path[count - 1] = end;
	count--;
	while (count>0) { // parent의 정점을 자동차가 가야할 path에 지정
		path[count - 1] = pre[temp];
		temp = pre[temp];
		count--;

	}

	before[0] = pre[temp];
	z = 0;


	while (1) { // 자동차의 방향을 알려주기 위한 direct배열 수정	
		first = before[z];
		second = path[z];
		third = path[z + 1];
		for (dircount = 0; dircount<100; dircount++) {
			if (road[dircount].f == first  && road[dircount].s == second && road[dircount].t == third) {
				direct[z] = road[dircount].dir;
				after[z] = third;
				break;
			}
		}
		before[z + 1] = path[z];
		if (path[z + 1] == end) { // 다음 path가 end일 경우
			break;
		}
		z++;
	}
	printf("----------------------------------------------------------\n");
	for (i = 0; i < num; i++) { printf("index :  %d , before_node : %d  ,node : %d  , after_node : %d  \n", i, before[i], path[i], after[i]); }
	printf("----------------------------------------------------------\n");
	for (i = 0; i < num; i++) { printf("index :  %d , direct : %c  \n", i, direct[i]); }
	printf("----------------------------------------------------------\n");


}

//배열들 초기화
void Clear() {
	int i = 0;
	for (i = 0; i<num; i++) {
		pre[i] = -1;
		weight[i] = 100000;
		direct[i] = 'A';
		path[i] = 100000;
		after[i] = 100000;
		before[i] = 100000;
		temparr[i] = 'A';
		temppath[i] = 100000;
	}
	direct[num + 1] = 'A';
	temparr[num + 1] = 'A';
}
//맵 초기화 및 다익스트라 재실행
void ClearMap()
{
	int i = 0, j = 0;
	int first = before[pathcnt - 1];//끊기기 전전 노드
	int second = before[pathcnt]; // 끊기기 전에 노드
	int third = path[pathcnt]; // 현재 노드
							   
	//first = 6;
	//second = 5;
	//third = 13;
	adjList[second][third] = 100000;
	adjList[third][second] = 100000;
	pathcnt = 0;
	Clear(); // 배열 초기화

	
	Dijkstra(second, 13);


	//시작점의 예외가 발생함으로 시작점에 대한 방향과 path를 가져오는 부분
	for (i = 0; i<sizeof(road) / sizeof(road[0]); i++) {
		if (road[i].f == third  && road[i].s == second && road[i].t == path[0]) {
			temparr[0] = road[i].dir;
			temppath[0] = second;
			break;
		}
	}

	printf("--------------tttttttttttttttt--------------\n");
	printf("%c", temparr[0]);
	


	for (i = 0; i < num; i++) {
		temparr[i + 1] = direct[i];
		temppath[i +1] = path[i];
		
	}
	for (j = 0; j <= num; j++) {
		direct[j] = temparr[j];
		path[j]= temppath[j];
	}
	printf("--------------wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww--------------\n");
	for (i = 0; i <= num; i++) { printf("index :  %d , direct : %c  \n", i, direct[i]); }
	for (i = 0; i <= num; i++) { printf("index :  %d , path : %d  \n", i, path[i]); }
	

}

void LineFind()//중앙을 잡기위한 부분
{
	while(1){		
		Line_Value= RoboCAR_Get_InfraredRay_Data();
		if(kbhit()) break;
		if(Line_Value==0xe7 || Line_Value==0xef || Line_Value==0xf7 || Line_Value==0xc3)
		{
			break;
		}
	}
}
