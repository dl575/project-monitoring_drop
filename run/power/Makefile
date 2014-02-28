.PHONY: results

results:
	./m5-mcpat.pl stats.txt config.ini mim.xml > output.xml
	mcpat -infile output.xml -print_level 5 > results
	rm run_one_results
	./parse_results.py results run_one_results

clean:
	rm -rf results run_one_results output.xml run_all_results.txt stats.* tables.* config.ini config.json mcpat.txt
