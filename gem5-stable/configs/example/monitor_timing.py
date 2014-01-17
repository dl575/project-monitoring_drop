#
# Include file for information about monitor timing
# This is AUTO GENERATED.
# Author: Mohamed Ismail
#
 
if model == 'ATOMIC':
	if monitor == 'UMC_FULL':
		full_wcet = 19
		drop_wcet = 19
	if monitor == 'UMC_SWDROP':
		full_wcet = 25
		drop_wcet = 5
	if monitor == 'UMC_HWDROP':
		full_wcet = 27
		drop_wcet = 1
	if monitor == 'UMC_HWFILTER':
		full_wcet = 24
		drop_wcet = 1
	if monitor == 'LRC_FULL':
		full_wcet = 17
		drop_wcet = 17
	if monitor == 'LRC_SWDROP':
		full_wcet = 22
		drop_wcet = 22
	if monitor == 'LRC_HWDROP':
		full_wcet = 21
		drop_wcet = 1
	if monitor == 'LRC_HWFILTER':
		full_wcet = 17
		drop_wcet = 1
	if monitor == 'DIFT_FULL':
		full_wcet = 26
		drop_wcet = 26
	if monitor == 'DIFT_SWDROP':
		full_wcet = 28
		drop_wcet = 16
	if monitor == 'DIFT_HWDROP':
		full_wcet = 41
		drop_wcet = 1
	if monitor == 'DIFT_HWFILTER':
		full_wcet = 30
		drop_wcet = 1
	if monitor == 'DIFT_RF_HWDROP':
		full_wcet = 39
		drop_wcet = 1
	if monitor == 'DIFT_RF_HWFILTER':
		full_wcet = 31
		drop_wcet = 1

if model == 'TIMING':
	if monitor == 'UMC_FULL':
		full_wcet = 30
		drop_wcet = 30
	if monitor == 'UMC_SWDROP':
		full_wcet = 44
		drop_wcet = 15
	if monitor == 'UMC_HWDROP':
		full_wcet = 55
		drop_wcet = 1
	if monitor == 'UMC_HWFILTER':
		full_wcet = 43
		drop_wcet = 1
	if monitor == 'LRC_FULL':
		full_wcet = 18
		drop_wcet = 18
	if monitor == 'LRC_SWDROP':
		full_wcet = 33
		drop_wcet = 33
	if monitor == 'LRC_HWDROP':
		full_wcet = 22
		drop_wcet = 1
	if monitor == 'LRC_HWFILTER':
		full_wcet = 18
		drop_wcet = 1
	if monitor == 'DIFT_FULL':
		full_wcet = 61
		drop_wcet = 61
	if monitor == 'DIFT_SWDROP':
		full_wcet = 63
		drop_wcet = 35
	if monitor == 'DIFT_HWDROP':
		full_wcet = 71
		drop_wcet = 1
	if monitor == 'DIFT_HWFILTER':
		full_wcet = 67
		drop_wcet = 1
	if monitor == 'DIFT_RF_HWDROP':
		full_wcet = 58
		drop_wcet = 1
	if monitor == 'DIFT_RF_HWFILTER':
		full_wcet = 50
		drop_wcet = 1

if model == 'FLEXHW':
	if monitor == 'UMC_HWFILTER':
		full_wcet = 8
		drop_wcet = 1
	if monitor == 'LRC_HWFILTER':
		full_wcet = 8
		drop_wcet = 1
	if monitor == 'DIFT_HWFILTER':
		full_wcet = 8
		drop_wcet = 1
	if monitor == 'DIFT_RF_HWFILTER':
		full_wcet = 8
		drop_wcet = 1

