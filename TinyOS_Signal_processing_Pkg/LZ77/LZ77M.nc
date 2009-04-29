#include "Timer.h"
#define DICTSIZE 250

#define SIZE  256
module LZ77M
{
 provides interface Lz77Compression;
 uses interface SplitControl as PrintfControl;
 uses interface PrintfFlush;
 uses interface Timer<TMilli>;
}
implementation
{
int matchlen,matchpos;
int dict[DICTSIZE];
int index1=0;
int nextlinklen=0;
int nextlink[DICTSIZE];
int Outbuffer[SIZE*3];
int FLAG =0;
int *arr;
int *brr;
int s;
uint16_t timetaken;
uint16_t start_time,end_time;


 int8_t search(int a)
 {
      int c,nextlinklen1=0;
      for(c=0;c<DICTSIZE;c++)
      {
           if(dict[c]==a)
             {
                nextlink[nextlinklen1++]=c;
                matchlen=1;
                FLAG++;

              }
            else
              {
                matchlen=0;
                matchpos=0;
                }
        }
       // printf("\nFlag=%d",FLAG);
       nextlinklen=nextlinklen1;
       nextlinklen1=0;
        if (FLAG>0)
        return 1;
 else return 0;
//call PrintfControl.start();
          }
void encode(int matchlen1,int matchpos1,int result,int symbol)

{
  if(index1>1996)
  index1=0;
  if(result==0)
  {
    Outbuffer[index1++]=0;
    Outbuffer[index1++]=symbol;
  }
  else
   {
   Outbuffer[index1++]=1;
   Outbuffer[index1++]=matchlen1;
   Outbuffer[index1++]=matchpos1;
   }
}
void shift(int val)

  {
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

void findMaxmatch()

{
//  call PrintfControl.start();
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
command error_t Lz77Compression.Lz77Encode(int *arr1)
{

 int i,j,k,m;
 brr=arr1;
 start_time=call Timer.getNow();
 for(j=0;j<DICTSIZE;j++)
 dict[j]=0xff;
  for (i=0;i<SIZE;i++)
     {

       k=search(brr[0]);
   

       if(k==1)
           {
              for(m=0;m<nextlinklen;m++)
              findMaxmatch();
              if (matchlen>=2)
              encode(matchlen,matchpos,k,brr[0]);
              else
              encode(matchlen,matchpos,0,brr[0]);

              shift(matchlen);
              i=i+matchlen-1;
              matchlen=0;

    
      }

        else
          {
           encode(matchlen,matchpos,k,brr[0]);
           shift(1);
           matchlen=0;


          }
     }

 end_time=call Timer.getNow();
  timetaken=end_time-start_time;
 signal Lz77Compression.Lz77EncodeDone(SUCCESS);
 // call PrintfControl.start();
  return SUCCESS;
}

 event void  PrintfControl.startDone(error_t err)
{  int j;
 printf("hello i am writing to u");
 printf("\n i am processing %d Bytes\n",sizeof(brr[0])*SIZE);
 printf("\n i am taking : %d seconds",timetaken);
// printf("\n matchlen:%d\n",matchlen)
 
// printf("\n matchposition %d\n",matchpos);
// for (j=0;j<SIZE;j++){printf("%d",dict[90+j]);}
  //         printf("\narr=");
          for (j=0;j<index1;j++){printf("%d", Outbuffer[j]);}

  call  PrintfFlush.flush();
}

event void  PrintfControl.stopDone(error_t err)
{
}
event void PrintfFlush.flushDone(error_t err)
{
}
event void Timer.fired()
{
}
}
