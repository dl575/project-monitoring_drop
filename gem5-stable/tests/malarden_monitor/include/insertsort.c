/* MDH WCET BENCHMARK SUITE. File version $Id: insertsort.c,v 1.3 2005/11/11 10:30:41 ael01 Exp $ */

/*************************************************************************/
/*                                                                       */
/*   SNU-RT Benchmark Suite for Worst Case Timing Analysis               */
/*   =====================================================               */
/*                              Collected and Modified by S.-S. Lim      */
/*                                           sslim@archi.snu.ac.kr       */
/*                                         Real-Time Research Group      */
/*                                        Seoul National University      */
/*                                                                       */
/*                                                                       */
/*        < Features > - restrictions for our experimental environment   */
/*                                                                       */
/*          1. Completely structured.                                    */
/*               - There are no unconditional jumps.                     */
/*               - There are no exit from loop bodies.                   */
/*                 (There are no 'break' or 'return' in loop bodies)     */
/*          2. No 'switch' statements.                                   */
/*          3. No 'do..while' statements.                                */
/*          4. Expressions are restricted.                               */
/*               - There are no multiple expressions joined by 'or',     */
/*                'and' operations.                                      */
/*          5. No library calls.                                         */
/*               - All the functions needed are implemented in the       */
/*                 source file.                                          */
/*                                                                       */
/*                                                                       */
/*************************************************************************/
/*                                                                       */
/*  FILE: insertsort.c                                                   */
/*  SOURCE : Public Domain Code                                          */
/*                                                                       */
/*  DESCRIPTION :                                                        */
/*                                                                       */
/*     Insertion sort for 10 integer numbers.                            */
/*     The integer array a[] is initialized in main function.            */
/*                                                                       */
/*  REMARK :                                                             */
/*                                                                       */
/*  EXECUTION TIME :                                                     */
/*                                                                       */
/*                                                                       */
/*************************************************************************/

/* Changes:
 * JG 2005/12/12: Indented program.
 */

#include "malarden.h"

int 
insertsort()
{

    INIT_MONITOR;

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
    
    DISABLE_MONITOR;
    while (!READ_FIFO_EMPTY);
    ENABLE_MONITOR;

    START_TASK(WCET_IS);
    
    START_SUBTASK(WCET_IS_1);
    
    
	while (i <= 10) {
		ENDSTART_SUBTASK(WCET_IS_2);
        
        j = i;
		while (a[j] < a[j - 1]) {
			temp = a[j];
			a[j] = a[j - 1];
			a[j - 1] = temp;
			j--;
		}
		i++;
	}
    
    END_SUBTASK;
    
    END_TASK(FIFO_SIZE*MON_DROP_WCET);
    
	return 1;
}
