using namespace cv;
using namespace std;
#define MAX_FEN 128

class FenProcessor {
	public:
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

	char* updateFen(char* fenString, char* move) {
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

		/*sections = strtok(NULL, " ");
		char* enpassant = checkEnPassant(chessBoard, move);

		sections = strtok(NULL, " ");
		char* halfMove = calculateHalfMove(chessBoard, move, sections);

		sections = strtok(NULL, " ");
		char* fullMove = sections;*/


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
		/*newFenString += enpassant + " ";

		//TODO: Half move update not implemented
		newFenString += halfMove + " ";

		if(turn.charAt(0) == 'b') newFenString += String.valueOf(Integer.parseInt(fullMove) + 1);
		else newFenString += fullMove;*/

		return newFenString;

	}

	private:
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

	int* processMove(char* move) {
		int startColIndex = charToIndex(move[0]);
		int startRowIndex = 7-charToIndex(move[1]);
		int endColIndex = charToIndex(move[2]);
		int endRowIndex = 7-charToIndex(move[3]);
		printf("MOVES: %d,%d %d,%d\n", startColIndex, startRowIndex, endColIndex, endRowIndex);

		int* moves = (int*) malloc(4 * sizeof(int));
		moves[0] = startColIndex; moves[1] = startRowIndex; 
		moves[2] = endColIndex; moves[3] = endRowIndex; 
		return moves;
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
			default: return '0';
		}
	}

	char* charToStr(char c) {
		char* cs = (char *) malloc(2 * sizeof(int));
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

	/*
	static private String calculateHalfMove(int[][] chessBoard, String move, String previousHalfMove) {
		int [] moveIndexes = processMove(move);
		int startColIndex = moveIndexes[0];
		int startRowIndex = moveIndexes[1];
		int endColIndex = moveIndexes[2];
		int endRowIndex = moveIndexes[3];

		boolean capture = chessBoard[endRowIndex][endColIndex] == 0;
		boolean pawn = chessBoard[startRowIndex][startColIndex] == R.drawable.white_pawn || chessBoard[startRowIndex][startColIndex] == R.drawable.black_pawn;
			
		if(capture && !pawn) {
			return String.valueOf(Integer.parseInt(previousHalfMove)+1);
		}
		return "0";
	}



	static private String checkEnPassant(int[][] chessBoard, String move) {
		int [] moveIndexes = processMove(move);
		int startColIndex = moveIndexes[0];
		int startRowIndex = moveIndexes[1];
		int endColIndex = moveIndexes[2];
		int endRowIndex = moveIndexes[3];

		int peice = chessBoard[startRowIndex][startColIndex];

		if(peice == R.drawable.white_pawn || peice == R.drawable.black_pawn) {
			if(startRowIndex == 1 || startRowIndex == 6) {
				if(Math.abs(startRowIndex - endRowIndex) == 2) {
					if(startRowIndex == 1) return move.substring(2,3) + "6";
					if(startRowIndex == 6) return move.substring(2,3) + "3";
				}
			}
		}

		return "-";
	}

	static private int charToPiece(char piece) {
		switch(piece) {
			case 'R': return R.drawable.white_rook;
			case 'N': return R.drawable.white_knight;
			case 'B': return R.drawable.white_bishop;
			case 'Q': return R.drawable.white_queen;
			case 'K': return R.drawable.white_king;
			case 'P': return R.drawable.white_pawn;
			case 'r': return R.drawable.black_rook;
			case 'n': return R.drawable.black_knight;
			case 'b': return R.drawable.black_bishop;
			case 'q': return R.drawable.black_queen;
			case 'k': return R.drawable.black_king;
			case 'p': return R.drawable.black_pawn;
			default: return 0;
		}
	}




	static private char pieceToChar(int piece) {
		switch(piece) {
			case R.drawable.white_rook: return 'R';
			case R.drawable.white_knight: return 'N';
			case R.drawable.white_bishop: return 'B';
			case R.drawable.white_queen: return 'Q';
			case R.drawable.white_king: return 'K';
			case R.drawable.white_pawn: return 'P';
			case R.drawable.black_rook: return 'r';
			case R.drawable.black_knight: return 'n';
			case R.drawable.black_bishop: return 'b';
			case R.drawable.black_queen: return 'q';
			case R.drawable.black_king: return 'k';
			case R.drawable.black_pawn: return 'p';
			default: return '0';
		}
	}

*/
};
