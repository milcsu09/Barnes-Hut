
CC := g++
# CCFLAGS := -std=c++17 -O3 $(CCWARNS)
CCFLAGS := -Wfatal-errors -Wall -Wextra -std=c++17 -Ofast -flto
CCFLAGS += -funroll-loops -fomit-frame-pointer -march=native
CCFLAGS += -fno-exceptions -fno-rtti -fopenmp
LDFLAGS := -lm -lsfml-graphics -lsfml-window -lsfml-system

OUTPUT := Barnes-Hut
$(OUTPUT): $(wildcard *.cc)
	$(CC) $(CCFLAGS) $^ -o $@ $(LDFLAGS)

