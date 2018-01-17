
LESSON5_SRC	:= hybrid_breadth_first.cpp main.cpp
LESSON5_HDR	:= hybrid_breadth_first.h

EXEC5		:= 05_trajectory_generation

all: $(EXEC5)
	./$(EXEC5)

$(EXEC5): $(LESSON5_SRC) $(LESSON5_HDR) Makefile
	g++ -I../../Eigen-3.3/Eigen $(LESSON5_SRC) -o $(EXEC5) -std=c++11

clean:
	rm -f $(EXEC5)
