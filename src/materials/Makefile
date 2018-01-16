
LESSON3_SRC	:= 03_17_naiveBayes.cpp 03_17_main.cpp
LESSON3_HDR	:= 03_17_naiveBayes.h
LESSON4_SRC	:= road.cpp vehicle.cpp cost.cpp main.cpp
LESSON4_HDR	:= road.h   vehicle.h   cost.h  

EXEC3		:= 03_17_naiveBayes
EXEC4		:= 04_behavior_planning

all: $(EXEC4)
	./$(EXEC4)

$(EXEC3): $(LESSON3_SRC) $(LESSON3_HDR) Makefile
	g++ -I../Eigen-3.3/Eigen $(LESSON3_SRC) -o $(EXEC3) -std=c++11

$(EXEC4): $(LESSON4_SRC) $(LESSON4_HDR) Makefile
	g++ -I../Eigen-3.3/Eigen $(LESSON4_SRC) -o $(EXEC4) -std=c++11

python:
	@#python 02_10_search.py
	@#python 02_11_search_expand.py
	@#python 02_12_search_policy.py
	@#python 02_14_search_astar.py
	@#python 02_19_search_dp.py
	@#python 02_20_search_optimum.py
	python 02_21_search_turnleft_policy.py

clean:
	rm -f $(EXEC3) $(EXEC4)
