/*
 * Combined file including multiple Malarden benchmarks, renamed as functions.
 */

#include "malarden.h"

/*** fac.c ***/
int 
fac(int n)
{
	if (n == 0)
		return 1;
	else
		return (n * fac(n - 1));
}

int 
factorial()
{
  START_TASK(WCET_FAC);

	int             i;
	int             s = 0;

  START_SUBTASK(WCET_FAC_SUB);
	for (i = 0; i <= 5; i++) {
		s += fac(i);
    ENDSTART_SUBTASK(WCET_FAC_SUB);
  }
  END_SUBTASK;

  END_TASK(FIFO_SIZE*MON_DROP_WCET);

	return (s);
}

/*** fibcall.c ***/
int 
fib(int n)
{
	int             i, Fnew, Fold, temp, ans;

	Fnew = 1;
	Fold = 0;

  START_SUBTASK(WCET_FC_SUB);
	for (i = 2;
	     i <= 30 && i <= n;	/* apsim_loop 1 0 */
	     i++) {
		temp = Fnew;
		Fnew = Fnew + Fold;
		Fold = temp;
    ENDSTART_SUBTASK(WCET_FC_SUB);
	}
  END_SUBTASK;
	ans = Fnew;
	return ans;
}

int 
fibcall()
{

  START_TASK(WCET_FC);

	int             a;

	a = 30;
	fib(a);

  END_TASK(FIFO_SIZE*MON_DROP_WCET);

	return a;
}


/*** insertsort.c ***/
int 
insertsort()
{
  START_TASK(WCET_IS);

	int             i, j, temp;

  unsigned int    a[11];
	a[0] = 0;		/* assume all data is positive */
	a[1] = 11;
	a[2] = 10;
	a[3] = 9;
	a[4] = 8;
	a[5] = 7;
	a[6] = 6;
	a[7] = 5;
	a[8] = 4;
	a[9] = 3;
	a[10] = 2;
	i = 2;
  
  START_SUBTASK(WCET_IS_1);
	while (i <= 10) {
		j = i;
		while (a[j] < a[j - 1]) {
			temp = a[j];
			a[j] = a[j - 1];
			a[j - 1] = temp;
			j--;
		}
		i++;
    ENDSTART_SUBTASK(WCET_IS_2);
	}
  END_SUBTASK;

  END_TASK(FIFO_SIZE*MON_DROP_WCET);

	return 1;
}


/** crc.c **/

unsigned short 
icrc1(unsigned short crc, unsigned char onech)
{
	int             i;
	unsigned short  ans = (crc ^ onech << 8);

	for (i = 0; i < 8; i++) {
        if (ans & 0x8000)
			ans = (ans <<= 1) ^ 4129;
		else
			ans <<= 1;
	}
    
	return ans;
}

unsigned short 
icrc(unsigned short crc, unsigned long len,
     short jinit, int jrev, unsigned char * lin)
{
	unsigned short  icrc1(unsigned short crc, unsigned char onech);
	static unsigned short icrctb[256], init = 0;
	static uchar    rchr[256];
	unsigned short  tmp1, tmp2, j, cword = crc;
	static uchar    it[16] = {0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15};

	if (!init) {
		init = 1;
		for (j = 0; j <= 255; j++) {
			ENDSTART_SUBTASK(WCET_CRC_3);

            icrctb[j] = icrc1(j << 8, (uchar) 0);
			rchr[j] = (uchar) (it[j & 0xF] << 4 | it[j >> 4]);
		}
	}
    
    ENDSTART_SUBTASK(WCET_CRC_4);
    
	if (jinit >= 0)
		cword = ((uchar) jinit) | (((uchar) jinit) << 8);
	else if (jrev < 0)
		cword = rchr[HIBYTE(cword)] | rchr[LOBYTE(cword)] << 8;
#ifdef DEBUG
	printf("len = %d\n", len);
#endif

	for (j = 1; j <= len; j++) {
    
        ENDSTART_SUBTASK(WCET_CRC_5);
    
		if (jrev < 0) {
			tmp1 = rchr[lin[j]] ^ HIBYTE(cword);
		} else {
			tmp1 = lin[j] ^ HIBYTE(cword);
		}
		cword = icrctb[tmp1] ^ LOBYTE(cword) << 8;
	}
    
    ENDSTART_SUBTASK(WCET_CRC_6);
    
	if (jrev >= 0) {
		tmp2 = cword;
	} else {
		tmp2 = rchr[HIBYTE(cword)] | rchr[LOBYTE(cword)] << 8;
	}
	return (tmp2);
}


int 
crc(void)
{

	unsigned short  i1, i2;
	unsigned long   n;
    unsigned char   lin[256] = "asdffeagewaHAFEFaeDsFEawFdsFaefaeerdjgp";

    START_TASK(WCET_CRC);
     
    START_SUBTASK(WCET_CRC_1);
    
	n = 40;
	lin[n + 1] = 0;
	i1 = icrc(0, n, (short) 0, 1, lin);
    
    ENDSTART_SUBTASK(WCET_CRC_2);
    
	lin[n + 1] = HIBYTE(i1);
	lin[n + 2] = LOBYTE(i1);
	i2 = icrc(i1, n + 2, (short) 0, 1, lin);
    
    END_SUBTASK;
    
    END_TASK(FIFO_SIZE*MON_DROP_WCET);
    
	return i2;
}

