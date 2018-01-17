
SRC	:= polynomial_solver.cpp
HDR	:= 

EXEC		:= 05_polynomial_solver

all: $(EXEC)
	./$(EXEC)

$(EXEC): $(SRC) $(HDR) Makefile
	g++ -I../../Eigen-3.3/Eigen $(SRC) -o $(EXEC) -std=c++11

clean:
	rm -f $(EXEC)
