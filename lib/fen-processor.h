using namespace cv;
using namespace std;
#define MAX_FEN 128

class FenProcessor {
	private:
	char* fenBoard;

	public:
	char* getFen() {
		return fenBoard;
	}
	void setFen(char* fen) {
		fenBoard = fen;
	}
	void updateFen(char* move) {
		char* fenString = fenBoard;
		char* chessBoard = processFen(fenString);
		for(int i = 0; i < 8; i++) {
			for(int j = 0; j < 8; j++) {
				printf("%c", chessBoard[i + 8 * j]);
			}
			printf("\n");
		}
		printf("\n");

		//Update board with move
		int* moveIndexes  = processMove(move);
		int startColIndex = moveIndexes[0];
		int startRowIndex = moveIndexes[1];
		int endColIndex = moveIndexes[2];
		int endRowIndex = moveIndexes[3];

		char* sections = strtok(fenString, " ");

		sections = strtok(NULL, " ");
		char* turn = sections;

		sections = strtok(NULL, " ");
		char* castling = checkCastling(chessBoard, move, sections);

		sections = strtok(NULL, " ");
		char* enpassant = checkEnPassant(chessBoard, move);

		sections = strtok(NULL, " ");
		char* halfMove = calculateHalfMove(chessBoard, move, sections);

		sections = strtok(NULL, " ");
		char* fullMove = sections;


		chessBoard[endRowIndex + 8 * endColIndex] = chessBoard[startRowIndex + 8 * startColIndex];
		chessBoard[startRowIndex + 8 * startColIndex] = '.';
		updateRookIfCastling(chessBoard, move);
		for(int i = 0; i < 8; i++) {
			for(int j = 0; j < 8; j++) {
				printf("%c", chessBoard[i + 8 * j]);
			}
			printf("\n");
		}
		printf("\n");
		char *newFenString = (char *) malloc(MAX_FEN * sizeof(char));
		strcpy(newFenString, "");
		for(int rowIndex = 0; rowIndex < 8; rowIndex++) {
			char row[MAX_FEN] = "";
			int spacing = 0;
			for(int colIndex = 0; colIndex < 8; colIndex++) {
				if(chessBoard[rowIndex + 8 * colIndex] != '.') {
					//printf("Processing: %c\n", chessBoard[rowIndex + 8 * colIndex]);
					//rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1

					if(spacing != 0) {
						strcat(row, charToStr(indexToChar(spacing)));
						spacing = 0;
					}

					strcat(row, charToStr(chessBoard[rowIndex + 8 * colIndex]));

				} else {
					spacing++;
					if(colIndex == 7) {
						strcat(row, charToStr(indexToChar(spacing)));
					}
				}
			}
			if(rowIndex < 7) strcat(row, "/");
			strcat(newFenString, row);
		}

		if(turn[0] == 'w') strcat(newFenString, " b ");
		else strcat(newFenString, " w ");

		strcat(newFenString, castling);
		strcat(newFenString, " ");
		strcat(newFenString, enpassant);
		strcat(newFenString, " ");
		strcat(newFenString, halfMove);

		char *newFullMove = (char *) malloc(5 * sizeof(char));
		strcpy(newFullMove, "");
		sprintf(newFullMove, " %d ", atoi(fullMove) + 1);
		if(turn[0] == 'b') strcat(newFenString, newFullMove);
		else strcat(newFenString, fullMove);

		fenBoard = newFenString;

		printf("RM New board: %s\n", fenBoard);
	}

	char getColourToMove() {
		char fenString[MAX_FEN] = ""; 
		strcpy(fenString, fenBoard);
		char* sections = strtok(fenString, " ");

		sections = strtok(NULL, " ");
		char* turn = sections;
		return turn[0];
	}

	char* indexToMove(int oldX, int oldY, int newX, int newY) {
		char * move = (char *) malloc(5 * sizeof(char));
		strcpy(move, "....");
		move[0] = rowIndexToChar(oldX + 1);
		move[1] = indexToChar(8 - oldY);
		move[2] = rowIndexToChar(newX + 1);
		move[3] = indexToChar(8 - newY);
		return move;
	}

	int* processMove(char* move) {
		int startColIndex = charToIndex(move[0]);
		int startRowIndex = 7-charToIndex(move[1]);
		int endColIndex = charToIndex(move[2]);
		int endRowIndex = 7-charToIndex(move[3]);
		printf("TRYING: %s\n", move);
		printf("WHAT IS: %c\n", move[0]);

		int* moves = (int*) malloc(4 * sizeof(int));
		moves[0] = startColIndex; moves[1] = startRowIndex; 
		moves[2] = endColIndex; moves[3] = endRowIndex; 
		return moves;
	}

	FenProcessor() {
		init();
	}
	private:
	void init() {
		fenBoard = (char*) malloc(MAX_FEN * sizeof(char));
		strcpy(fenBoard, "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
	}
	char* processFen(char* fenString) {
		char cFenString[MAX_FEN];
		strcpy(cFenString, fenString);

		char *newChessBoard = (char *) malloc(65 * sizeof(char));
		strcpy(newChessBoard, "................................................................");
			
		char* board = strtok(cFenString, " ");
		char* pieces = strtok(board, "/");
		int rowIndex = 0;
		while( pieces != NULL) {
			int pieceIndex = 0;
			for(int index = 0; index < strlen(pieces); index++) {
				char piece = pieces[index];
				newChessBoard[rowIndex + 8 * pieceIndex] = charToPiece(piece);
				pieceIndex += charToSpacing(piece); //If piece index is 8 or greater program should auto terminate.  Not checked so watch this line
			}
			rowIndex++;
			pieces = strtok(NULL, "/");
		}
		return newChessBoard;
	}
	int charToPiece(char piece) {
		switch(piece) {
			case 'R': return piece;
			case 'N': return piece;
			case 'B': return piece;
			case 'Q': return piece;
			case 'K': return piece;
			case 'P': return piece;
			case 'r': return piece;
			case 'n': return piece;
			case 'b': return piece;
			case 'q': return piece;
			case 'k': return piece;
			case 'p': return piece;
			default: return '.';
		}
	}

	int charToSpacing(char piece) {
		switch(piece) {
			case '2': return 2;
			case '3': return 3;
			case '4': return 4;
			case '5': return 5;
			case '6': return 6;
			case '7': return 7;
			case '8': return 8;

			default: return 1;
		}
	}

	int charToIndex(char col) {
		switch(col) {
			case 'a': return 0;
			case 'b': return 1;
			case 'c': return 2;
			case 'd': return 3;
			case 'e': return 4;
			case 'f': return 5;
			case 'g': return 6;
			case 'h': return 7;
				  
			case '1': return 0;
			case '2': return 1;
			case '3': return 2;
			case '4': return 3;
			case '5': return 4;
			case '6': return 5;
			case '7': return 6;
			case '8': return 7;
			

			default: return -1;
		}
	}

	char indexToChar(int piece) {
		switch(piece) {
			case 1: return '1';
			case 2: return '2';
			case 3: return '3';
			case 4: return '4';
			case 5: return '5';
			case 6: return '6';
			case 7: return '7';
			case 8: return '8';
			default: return 0;
		}
	}

	char rowIndexToChar(int piece) {
		switch(piece) {
			case 1: return 'a';
			case 2: return 'b';
			case 3: return 'c';
			case 4: return 'd';
			case 5: return 'e';
			case 6: return 'f';
			case 7: return 'g';
			case 8: return 'h';
			default: return 0;
		}
	}

	char* charToStr(char c) {
		char* cs = (char *) malloc(2 * sizeof(char));
		strcpy(cs, " ");
		cs[0] = c;
		return cs;
	}

	char* checkCastling(char* chessBoard, char* move, char* castling) {
		int* moveIndexes = processMove(move);
		int startColIndex = moveIndexes[0];
		int startRowIndex = moveIndexes[1];
		int endColIndex = moveIndexes[2];
		int endRowIndex = moveIndexes[3];


		bool whiteKing = strstr(castling, "K") != NULL;
		bool whiteQueen = strstr(castling, "Q") != NULL;
		bool blackKing = strstr(castling, "k") != NULL;
		bool blackQueen = strstr(castling, "q") != NULL;
		char peice = chessBoard[startRowIndex + 8 * startColIndex];
		if(peice == 'K') {
			whiteKing = false;
			whiteQueen = false;
		} else if(peice == 'R') {
			if(startColIndex == 7) whiteKing = false;
			else if(startColIndex == 0) whiteQueen = false;
		}

		if(peice == 'k') {
			blackKing = false;
			blackQueen = false;
		} else if(peice == 'r') {
			if(startColIndex == 7) blackKing = false;
			else if(startColIndex == 0) blackQueen = false;
		}

		char *newCastling = (char *) malloc(5 * sizeof(char));
		strcpy(newCastling, "");
		if(!whiteKing && !whiteQueen && !blackKing && !blackQueen) {
			strcpy(newCastling, "-");
			return newCastling;
		}

		if(whiteKing) strcat(newCastling, "K");
		if(whiteQueen) strcat(newCastling, "Q");
		if(blackKing) strcat(newCastling, "k");
		if(blackQueen) strcat(newCastling, "q");
		
		return newCastling;
	}

	char* updateRookIfCastling(char* chessBoard, char* move) {
		int* moveIndexes = processMove(move);
		int startColIndex = moveIndexes[0];
		int startRowIndex = moveIndexes[1];
		int endColIndex = moveIndexes[2];
		int endRowIndex = moveIndexes[3];

		char peice = chessBoard[endRowIndex + 8 * endColIndex];
		bool blackWhite = peice == 'k' ? true : false;

		if(peice == 'K' || peice == 'k') {
			if(abs(startColIndex - endColIndex) == 2) {
				//Castling
				if(endColIndex > 4) {
					//King side
					if(blackWhite) {
						chessBoard[startRowIndex + 8 * 5] = 'r';
						chessBoard[startRowIndex + 8 * 7] = 0;
					} else {
						chessBoard[startRowIndex + 8 * 5] = 'R';
						chessBoard[startRowIndex + 8 * 7] = 0;
					}
				} else {
					//Queen side
					if(blackWhite) {
						chessBoard[startRowIndex + 8 * 3] = 'r';
						chessBoard[startRowIndex + 8 * 0] = 0;
					} else  {
						chessBoard[startRowIndex + 8 * 3] = 'R';
						chessBoard[startRowIndex + 8 * 0] = 0;
					}
				}

				return chessBoard;
			}
		}

		return chessBoard;
	}

	char* checkEnPassant(char* chessBoard, char* move) {
		int* moveIndexes = processMove(move);
		int startColIndex = moveIndexes[0];
		int startRowIndex = moveIndexes[1];
		int endColIndex = moveIndexes[2];
		int endRowIndex = moveIndexes[3];
		char *enpassant = (char*) malloc(4 * sizeof(char));
		strcpy(enpassant, "..");

		char peice = chessBoard[startRowIndex + 8 * startColIndex];

		if(peice == 'P' || peice == 'p') {
			if(startRowIndex == 1 || startRowIndex == 6) {
				if(abs(startRowIndex - endRowIndex) == 2) {
					enpassant[0] = move[2];
					if(startRowIndex == 1) {
						enpassant[1] = '6';
						return enpassant;
					}
					if(startRowIndex == 6) {
						enpassant[1] = '3';
						return enpassant;
					}
				}
			}
		}

		strcpy(enpassant, "-");
		return enpassant;
	}

	char* calculateHalfMove(char* chessBoard, char* move, char* previousHalfMove) {
		int* moveIndexes = processMove(move);
		int startColIndex = moveIndexes[0];
		int startRowIndex = moveIndexes[1];
		int endColIndex = moveIndexes[2];
		int endRowIndex = moveIndexes[3];

		bool capture = chessBoard[endRowIndex + 8 * endColIndex] == 0;
		bool pawn = chessBoard[startRowIndex + 8 * startColIndex] == 'p' || chessBoard[startRowIndex + 8 * startColIndex] == 'P';
			
		char* halfMove = (char *) malloc(8 * sizeof(char));
		strcpy(halfMove, "");
		if(capture && !pawn) {
			sprintf(halfMove, "%d", (atoi(previousHalfMove)+1)); 
			return halfMove;
		}
		strcpy(halfMove, "0");
		return halfMove;
	}
};
