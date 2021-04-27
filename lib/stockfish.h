using namespace cv;
using namespace std;

class Stockfish {
	private:
	int input[2];
	int output[2];

	public:
	Stockfish() { init(); }

	void clearBuffer() {
		char buffer[BUFSIZ];
		if(read(output[0], buffer, BUFSIZ) > 0) {
			printf("BUFFER: %s\n", buffer);
		}
	}

	void startFenCalc(char *fen) {
		char move[BUFSIZ] = "position fen ";
		strcat(move, fen);
		strcat(move, "\ngo\n");
		write(input[1], move, strlen(move));
	}

	void stopCalc() {
		char cmd[BUFSIZ] = "stop\n";
		write(input[1], cmd, strlen(cmd));
	}

	char* readBestMove() {
		char buffer[BUFSIZ];
		while(read(output[0], buffer, BUFSIZ) > 0) {
			char *bestMove = strstr(buffer, "bestmove");
			if(bestMove != NULL) {
				char* move = bestMove + strlen("bestmove");
				move[5] = '\0';
				return bestMove;
			}
		}
		return NULL;
	}

	private:
	bool init() {
		pid_t p;
		if(pipe(input)==-1) {
			fprintf(stderr, "Pipe Failed");
			return false;
		}	
		if(pipe(output)==-1) {
			fprintf(stderr, "Pipe Failed");
			return false;
		}


		p = fork();
		if(p < 0) { 
			fprintf(stderr, "Fork failed");
			return false;
		}
		if( p > 0 ) {
			//Parent
			close(input[0]);
			close(output[1]);

			char isReady[] = "isready\n";
			write(input[1], isReady, strlen(isReady));
		} else {
			//Child
			close(input[1]);
			close(output[0]);

			dup2(input[0], STDIN_FILENO);
			dup2(output[1], STDOUT_FILENO);
			stockfish();
		}

		return true;
	}



	void stockfish() {
		char cmd[] = "/home/alasdair/stockfish/stockfish_13_linux_x64/stockfish_13_linux_x64";
		char *args[] = {cmd, NULL};
		execvp(cmd, args);
		while(true) {
			fflush(stdin);
			fflush(stdout);
		}

	}
};
