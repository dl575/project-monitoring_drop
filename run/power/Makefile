.PHONY: results

results:
	./m5-mcpat.pl stats.txt config.ini mim.xml > output.xml
	mcpat -infile output.xml -print_level 5 > results
