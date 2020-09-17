#ifndef  _BUF_H
#define  _BUF_H

typedef  struct
{               
	uint8_t       *Start;        //定位在缓存开始           
	uint8_t       *QEnd;         //定位在缓存结束   
	uint8_t       *OSQIn;        //当前写入位置 
	uint8_t       *OSQOut;       //当前读出位置
	uint8_t       OSQSize;       //缓存总的大小 
	uint8_t     OSQEntries;      //当前缓存中存储的字节数 现在要考虑缓存覆盖问题
} buf_type;  //定义跟踪缓存的数据结构

typedef enum{ EMPTY=0,OK=1,FULL=2}buf;

extern void       buf_init(buf_type * _os,uint8_t  *_Event,uint8_t buf_size) ;  
extern void       buf_write(buf_type * _os,uint8_t *p);
extern  uint8_t   buf_read(buf_type * _os,uint8_t *p) ;

// extern __ASM  void __enter_critcal(void);
// extern __ASM  void __exit_critcal(void);


#endif
