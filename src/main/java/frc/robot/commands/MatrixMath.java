package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MatrixMath extends CommandBase{
    public int row, col;
    public double[][] dataSet;

    public MatrixMath(int numRow, int numCol) {
        // this.dataSet = new double[numRow][numCol];
        // for (int i = 0; i < numRow; i++) {
        //     for (int j = 0; j < numCol; j++) {
        //         dataSet[i][j] = 0;
        //     }
        // }

        // for(double[] row : this.dataSet){
        //     for(double cell : row){

        //     }
        // }

        row = numRow;
        col = numCol;
    }

    public static void matrixAddition(MatrixMath x, MatrixMath y) {
        if (x.row == y.row && x.col == y.col) {
            MatrixMath resultantMatrix = new MatrixMath(x.row, x.col);

        }
        else {
            //throw error
        }

        
    }

    

    public String toString() {
        String matrixString = "";

        // for (int i = 0; i < row; i++) {
        //     for (int j = 0; j < col; j++) {
        //         matrixString += dataSet[row][col] + ", ";
        //     }
        //     matrixString += "\n";
        // }
        matrixString += row + col;
        return matrixString;
    }
}
