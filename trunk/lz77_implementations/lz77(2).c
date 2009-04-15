/*LZ77 Compession algorithm by Haris Alatas(alatasst@otenet.gr). This is a project for the
  course "Data Compression" lead by Dr. I. Rekanos at T.E.I. of Serres, Greece.
  Because of the way(in order the human eye to be able to read it) the algorithm saves to disk 
  the coded data is not compressed.
  I think that the program is bug free. If you find any please contact me. There have been no optimizations
  so there are many lines that are useless. 
  Please dont try to code bigs file because the way it read data from the disk is a bit nasty.
  I dont take any responsibilities about the damage that the proggy could do in your data or hardware.
  You can find this little agorithm at http://users.otenet.gr/~alatasst Feel free to write your comments at alatasst@otenet.gr
  I have only tried to compile it in GNU/linux on a x86.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ALLOC_AT_EMPTY_MATRIX -1 //DONT CHANGE
#define AUTO_REMOVE_FILENAME 1 //IF 1: DELETES THE TARGET FILE
#define START_FROM_OFFSET 0 //STARTING OFFSET IS 0
#define CHAR_OR_INT 0 //0 TO SAVE CHARS - 1 TO SAVE ASCII NUM OF CHARS AT THE LAST FIELD
#define PUT_NO_DICT 0 //IF 1 THEN DICT WILL NOT BE USED
#define DEBUG_MODE  0 //1 FOR DEBUG MODE

long lenoffile(FILE *source);
void display_usage(char *fname);
void wrong_dict();
void wrong_l_a();
long compress(char *dict,char *lab,FILE *target,long dict_size,long lab_size);
long searchfor(char *dict,char lab[1],long dict_pos,long dict_size);
int fileexists(char *filename);
void fill_dict(char *dict,FILE *source,long dict_size);
void fill_lab(FILE *source,char *lab,long lab_size);
void null_t_m(long *matrix,long size);
long compstrings(char *dict,char *lab,long dict_pos,long lab_pos,long dict_size,long lab_size);
long flop(float *flop,long *matched,long *matrix);

int main (int argc, char *ch[])
{
   char *filename,*dict,*lab;
   char extens[]=".lz";
   FILE *source;
   FILE *target;
   long dict_size,lab_size,i,dict_sz,lab_sz,moveto,lof;
   
   if (argc<4) display_usage(ch[0]);
   if (argc>=2) if(atoi(ch[1])<1) wrong_dict();
   if (argc>=3) if(atoi(ch[2])<1) wrong_l_a();	 
   if (argc>=4) 
     {
      	if ((source=fopen(ch[3],"rb"))==NULL)
	  {
	     printf("\nError opening source file\n\n");
	     exit(0);
	  }
	
	filename=malloc(strlen(ch[3])+4);
	strcpy(filename,ch[3]);
	strcat(filename,extens);
	if (AUTO_REMOVE_FILENAME==1)if (remove(filename)==0) printf("FILE REMOVED\n");
	if (!fileexists(filename))
	  {
	     
	if ((target=fopen(filename,"w"))==NULL)
	  {
	     printf("Could not open target file: %s\nExiting.\n",filename);
	     exit(0);
	  }
	  }
	else
	  {
	     printf("Target file %s already exists.\n",filename);
	     exit(0);
	  }
        dict_size=atol(ch[1]);
	lab_size=atol(ch[2]);
        dict=malloc((sizeof(char)*(int)dict_size)-1);
       	if (lab_size>=lenoffile(source))lab=malloc((sizeof(char)*(int)lenoffile(source))-1);
	else lab=malloc((sizeof(char)*(int)lab_size)-1);
	
	lof=lenoffile(source);
	fseek(source,sizeof(char)*(int)START_FROM_OFFSET,0);
	fill_lab(source,lab,1);
	if(CHAR_OR_INT==0) fprintf(target,"(0,0,%c)\n",lab[0]);
	else fprintf(target,"(0,0,%i)\n",lab[0]);
	//fseek(source,sizeof(char)*(int)START_FROM_OFFSET,0);
	i=1;
	while (i<lof)
	  {
	     if (dict_size>ftell(source)) dict_sz=ftell(source);
	     else dict_sz=dict_size;
	     if (lab_size>(lenoffile(source)-ftell(source))) lab_sz=lenoffile(source)-ftell(source);
	     else lab_sz=lab_size;
	     
	     fill_dict(dict,source,dict_sz);

       	     fill_lab(source,lab,lab_sz);
	     if(DEBUG_MODE==1) printf("LAB:%s\nLABSIZE:%i\n",lab,lab_sz);
	     fseek(source,sizeof(char)*(int)i,0);
	     
	     moveto=compress(dict,lab,target,dict_sz,lab_sz);
	     i=i+moveto;
	     fseek(source,sizeof(char)*(int)i,0);
	     
	  }
	fclose(source);
    
     }
   
   
}

long lenoffile(FILE *source)
{
   long curpos,lof;
   curpos=ftell(source);
   fseek(source,0,2);
   lof=ftell(source);
   fseek(source,curpos,0);
   return(lof);
}

void display_usage(char *fname)
{
   printf("LZ77 compression algorithm. By Haris Alatas(alatasst@otenet.gr).\n");
   printf("\nUsage: lz277 <dictionary> <look_ahead> filename");
   printf("\n\n");
   printf("For example: If you want to compress the file \"uncmopress.o\"\n");
   printf("with look_ahead=5 and dictionary=6 then you must type:");
   printf("%s 6 5 uncompress.o\n\n",fname);
   exit(0);
}

void wrong_dict()
{
   printf("No reason to declare null dictionary\nExiting..\n\n");
   exit(0);
}

void wrong_l_a()
{
   printf("No reason to declare null lookahead buffer\nExiting..\n\n");
   exit(0);
}

long compress(char *dict,char *lab,FILE *target,long dict_size,long lab_size)
{
   char pdict[dict_size];
   char plab[lab_size];
   long i,j,temp;
   long matrix[dict_size];
   long matched[dict_size];
   long dict_sz,magicnum;
   float tflops[dict_size];
   
   
   dict_sz=dict_size;
   j=0;
   strcpy(pdict,dict);
   strcpy(plab,lab);
   
   null_t_m(matrix,dict_size);
   null_t_m(matched,dict_size);
   
   
   for(i=0;i<dict_size;i++)
     { 
	matrix[j]=searchfor(pdict,plab,i,dict_sz);
        if(matrix[j]!=ALLOC_AT_EMPTY_MATRIX)
	  {
	     i=matrix[j];
	     j++;
	  }
	
     }
   j=0;
   i=0;
   while(i<dict_size)
     {
	if (matrix[i]!=ALLOC_AT_EMPTY_MATRIX)
	  {
	     matched[i]=compstrings(pdict,plab,matrix[i]+1,1,dict_size,lab_size)+1;
	     
	     matrix[i]=dict_size-matrix[i];
	  }
	i++;
     }
   j=flop(tflops,matched,matrix);
   
   magicnum=0;
   for(i=0;i<(j+PUT_NO_DICT);i++)
     {
	if (tflops[magicnum]>tflops[i]) magicnum=i;
	if(DEBUG_MODE==1) printf("(%i-%i-%.3f)\n",matrix[i],matched[i],tflops[i]);
     }
   if(DEBUG_MODE==1) printf("CHOOSED:(%i-%i-%.3f)\n",matrix[magicnum],matched[magicnum],tflops[magicnum]);
   if (matrix[magicnum]!=-1)
     {
	if (CHAR_OR_INT==0)fprintf(target,"(%i,%i,%c)\n",matrix[magicnum],matched[magicnum],plab[matched[magicnum]]);
	else fprintf(target,"(%i,%i,%i)\n",matrix[magicnum],matched[magicnum],plab[matched[magicnum]]);
	return(matched[magicnum]+1);
     }
   else
     {
	if (CHAR_OR_INT==0)fprintf(target,"(%i,%i,%c)\n",0,0,plab[0]);
	else fprintf(target,"(%i,%i,%i)\n",0,0,plab[0]);
	return(1);
     }
   
   
}


long searchfor(char *dict,char lab[1],long dict_pos,long dict_size)
{
   long i;
   char pdict[dict_size];
   char plab[1];
   //long dict_ps;
   
   strcpy(pdict,dict);
   plab[0]=lab[0];  
   for (i=(dict_pos);i<dict_size;i++)
     {
	if (pdict[i]==plab[0])
	  {
	     return i;
	  }
     }
   return ALLOC_AT_EMPTY_MATRIX;
}

int fileexists(char *filename)
{
  FILE *target;   
   
   if((target=fopen(filename,"r"))==NULL)
     {
	return(0);
     }
   else
     {
	fclose(target);
	return(1);
     }
     
   
}
void fill_dict(char *dict,FILE *source,long dict_size)
{
   char *phar;
   char *ptemp1;
   long offset,i,dict_sz;
   phar=&dict[0];
   dict_sz=dict_size;
   offset=ftell(source);
   ptemp1=malloc(sizeof(char));
   //if(dict_sz>offset) dict_sz=offset;
   fseek(source,sizeof(char)*(-dict_sz),1);
   
   for(i=0;i<dict_sz;i++)
     {
	fread(ptemp1,sizeof(char),1,source);
	phar[i]=*ptemp1;
	
     }
   //dict=malloc(sizeof(char)*(int)dict_size);
   dict=&phar[0];
}

void fill_lab(FILE *source,char *lab,long lab_size)
{
   char *plab,temp[1];
   long i,lof;
   lof=lab_size;
   plab=&lab[0];
   for(i=0;i<lof+1;i++)
     {
	plab[i]=' ';
     }
   
	
   for(i=0;i<lof;i++)
     {
	if(fread(temp,sizeof(char),1,source)!=1)
	  {
	     printf("Error reading file..\nExiting.\n");
	     exit(1);
	  }
	
        plab[i]=temp[0];
     }
   plab[i+1]='\0';
   
   
}
   
       
void null_t_m(long *matrix,long size)
{
   long i;
   long *mat;
   mat=&matrix[0];
   for (i=0;i<size;i++)
     {
	mat[i]=ALLOC_AT_EMPTY_MATRIX;
     }
}

long compstrings(char *dict,char *lab,long dict_pos,long lab_pos,long dict_size,long lab_size)
{
   char pdict[(dict_size)];
   char tdict[dict_size+lab_size];
   char plab[lab_size];
   long i;
   
   if((lab_pos+1)==lab_size) return (0);
   strcpy(pdict,dict);
   strcpy(tdict,pdict);
   strcat(tdict,lab);
   strcpy(plab,lab);
   if(tdict[dict_pos]==plab[lab_pos])
     {
	return (1+compstrings(dict,lab,dict_pos+1,lab_pos+1,dict_size,lab_size));
     }
   if(DEBUG_MODE==1) printf("TDICT:%s",tdict);
   if(DEBUG_MODE==1) printf("LAB:%sLABSIZE:%i\n",lab,lab_size);
   return (0);
}

long flop(float *flop,long *matched,long *matrix)
{
   float *pfloat;
   long *pmatched;
   long *pmatrix;
   long i,j;
   i=0;
   pfloat=&flop[0];
   pmatched=&matched[0];
   pmatrix=&matrix[0];
   
   while(pmatrix[i]!=-1)
     {
	pfloat[i]=(float)pmatrix[i]/(float)pmatched[i];
	i++;
     }
   return(i);
}

   

