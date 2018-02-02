
all:
	~/.pyenv/shims/watchmedo shell-command --patterns="*.cpp" --command='(cd ../build; make;)' .

run:
	(cd ../build; make; ./path_planning)
