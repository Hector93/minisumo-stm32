all:
	$(MAKE) lib
#	$(MAKE) test

lib:
	arm-none-eabi-c++ -c -Os  -Wall --specs=nosys.specs -mthumb -mcpu=cortex-m3 -specs=nano.specs ./*.cpp
	arm-none-eabi-c++ -c -Os  -Wall --specs=nosys.specs -mthumb -mcpu=cortex-m3 -specs=nano.specs *.cpp

test:
	arm-none-eabi-c++ ./tests/GeneralTest.cpp Fuzzy.o FuzzyComposition.o FuzzyIO.o FuzzyInput.o FuzzyOutput.o FuzzyRule.o FuzzyRuleAntecedent.o FuzzyRuleConsequent.o FuzzySet.o -o tests/GeneralTest.bin -fPIC -O2 -g -Wall --specs=nosys.specs
	arm-none-eabi-c++ ./tests/FuzzyTest.cpp Fuzzy.o FuzzyComposition.o FuzzyIO.o FuzzyInput.o FuzzyOutput.o FuzzyRule.o FuzzyRuleAntecedent.o FuzzyRuleConsequent.o FuzzySet.o /usr/lib/libgtest.a -o tests/FuzzyTest.bin -fPIC -O2 -g -Wall -lpthread --specs=nosys.specs

clean:
	rm -f *.o
	rm -f ./examples/*/*.bin
	rm -f ./tests/*.bin
	rm -rf ./tests/*.bin.*
