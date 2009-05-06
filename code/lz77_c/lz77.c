#include <stdio.h>

#define DICTSIZE 256
#define SIZE  32


int matchlen,matchpos;
int dict[DICTSIZE];
int outBufferIndex=0;
int nextlinklen=0;
int nextlink[DICTSIZE];
int Outbuffer[SIZE*3];
int FLAG =0;
int *data;
int *brr;
int s;
int timetaken;
int start_time,end_time;


int search(int a){
  int c,nextlinklen1=0;
  for(c=0;c<DICTSIZE;c++) {
    if(dict[c]==a) {
      nextlink[nextlinklen1++]=c;
      matchlen=1;
      FLAG++;
      
    } else {
      matchlen=0;
      matchpos=0;
    }
  }
  // printf("\nFlag=%d",FLAG);
  nextlinklen=nextlinklen1;
  nextlinklen1=0;
  
  if (FLAG>0)
    return 1;
  else 
    return 0;
}

void encode(int matchlen1,int matchpos1,int result,int symbol) {

  printf("\nencode\n");
  printf("matchlen1: %d\n", matchlen1);
  printf("matchpos1: %d\n", matchpos1);
  printf("result: %d\n", result);
  printf("symbol: %d\n", symbol);
  printf("OutBufferIndex: %d\n", outBufferIndex);
  
  if(outBufferIndex>1996)
    outBufferIndex=0;
  if(result==0) {
    Outbuffer[outBufferIndex++]=0;
    Outbuffer[outBufferIndex++]=symbol;
  } else {
    Outbuffer[outBufferIndex++]=1;
    Outbuffer[outBufferIndex++]=matchlen1;
    Outbuffer[outBufferIndex++]=matchpos1;
  }
}

void shift(int val) {
  //    rand();
  int m,n,val2,val3;
  val3=val2=val;
  //    rand();
  do{
    for(m=0;m<DICTSIZE-1;m++)
      dict[m]=dict[m+1];
    
  }
  while((--val2)!=0) ;
  
  for(n=0;n<val;n++)
    dict[DICTSIZE-val+n]=brr[n];
  do{
    for(n=0;n<SIZE-1;n++)
      brr[n]=brr[n+1];
    brr[SIZE-1]=0xFF;
  }
  while((--val3)!=0);
  
}

void printDict() {
  int i;
  printf("Dictionary:\n");
  for (i=0; i<DICTSIZE; i++) {
    printf("%d ", dict[i]);
  }
  printf("\n\n");
}

void findMaxmatch() {
  
  int n=0,m,p;
  int nindex=1;
  do{
    for( m=0,p=0;m<nextlinklen;m++)
      {
	if(nextlink[m]==DICTSIZE-1)
	  break;
	else
	  {
	    
	    if(brr[nindex]==(dict[nextlink[m]+nindex]))
	      {
		
		nextlink[p++]=nextlink[m];
	      }
	    
	  }
      }
    nindex++;
    nextlinklen=p;
  }
  while(p!=0);
  matchpos=nextlink[0];
  matchlen=nindex-1;
}
int* Lz77Encode(int *data) {
  
  int i,j,k,m;

  for (i = 0; i < SIZE*3; i++) {
    Outbuffer[i] = 256;
  }

  
  brr=data;
  start_time=0;
  for(j=0;j<DICTSIZE;j++)
    dict[j]=0xff;
  for (i=0;i<SIZE;i++) {
    
    printDict();
    k=search(brr[0]);
    
    
    if(k==1) {
      for(m=0;m<nextlinklen;m++)
	findMaxmatch();
      if (matchlen>=2)
	encode(matchlen,matchpos,k,brr[0]);
      else
	encode(matchlen,matchpos,0,brr[0]);
      
      shift(matchlen);
      i=i+matchlen-1;
      matchlen=0;
      
      
    } else {
      encode(matchlen,matchpos,k,brr[0]);
      shift(1);
      matchlen=0;
      
      
    }
  }
  
  end_time=0;
  timetaken=end_time-start_time;

  return Outbuffer;
}


int main(int argc, char** argv) {

  int buffer[SIZE];
  int i;

  printf("Input Data:\n");
  for (i = 0; i < SIZE; i++) {
    buffer[i] = i%4;
    printf("%d ", buffer[i]);
  }
  printf("\n");
  
  
  Lz77Encode(buffer);

  //printf("\nData...\n");
  for (i = 0; i < outBufferIndex; i++) {
    printf("%d\n", Outbuffer[i]);
  }
  printf("256");
  //printf("--------\n");


  return 0;
}
