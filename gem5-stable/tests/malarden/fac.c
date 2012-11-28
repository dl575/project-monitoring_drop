/* MDH WCET BENCHMARK SUITE. File version $Id: fac.c,v 1.1 2005/11/11 10:14:54 ael01 Exp $ */

 /*
  * Changes: JG 2005/12/23: Inserted prototype.
  *                         Indented program.
  */
int             fac(int n);

int 
fac(int n)
{
	if (n == 0)
		return 1;
	else
		return (n * fac(n - 1));
}

int 
main(void)
{
  int *fifo;
  fifo = (int *)0x30000000;

  *fifo = 1;

	int             i;
	int             s = 0;

	for (i = 0; i <= 5; i++)
		s += fac(i);

  *fifo = 0;

  while(1);
  
	return (s);
}
