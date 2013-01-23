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
  START_TASK(0);

	int             i;
	int             s = 0;

  START_SUBTASK(WCET_FAC);
	for (i = 0; i <= 5; i++) {
		s += fac(i);
    ENDSTART_SUBTASK(WCET_FAC);
  }
  END_SUBTASK(WCET_FAC);

  END_TASK(FIFO_SIZE*MON_DROP_WCET);

	return (s);
}

/*** insertsort.c ***/
int 
insertsort()
{
  START_TASK(0);

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
  
  START_SUBTASK(WCET_IS);
	while (i <= 10) {
		j = i;
		while (a[j] < a[j - 1]) {
			temp = a[j];
			a[j] = a[j - 1];
			a[j - 1] = temp;
			j--;
		}
		i++;
    ENDSTART_SUBTASK(WCET_IS);
	}
  END_SUBTASK;

  END_TASK(FIFO_SIZE*MON_DROP_WCET);

	return 1;
}

/*** fibcall.c ***/
int 
fib(int n)
{
	int             i, Fnew, Fold, temp, ans;

	Fnew = 1;
	Fold = 0;

  START_SUBTASK(WCET_FC);
	for (i = 2;
	     i <= 30 && i <= n;	/* apsim_loop 1 0 */
	     i++) {
		temp = Fnew;
		Fnew = Fnew + Fold;
		Fold = temp;
    ENDSTART_SUBTASK(WCET_FC);
	}
  END_SUBTASK;
	ans = Fnew;
	return ans;
}

int 
fibcall()
{

  START_TASK(0);

	int             a;

	a = 30;
	fib(a);

  END_TASK(FIFO_SIZE*MON_DROP_WCET);

	return a;
}
