package frc.lib.util;

import java.util.ArrayList;

public final class InterpolatingDoubleTable {
    private ArrayList<InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>> tbl_list 
            = new ArrayList<InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>>();

    /**
     * Default Constructor of an interpolating double table
     * @param columns the number of columns needed in the table (including your reference column)
     */        
    public InterpolatingDoubleTable(int columns) {
        for (int i = 0; i <= columns - 2; i++) {
            tbl_list.add(new InterpolatingTreeMap<>());
        }
    }

    /**
     * Add a reference point in the table
     * @param ref the reference value
     * @param vals values of the other columns at this reference value
     */
    public void addPoint(double ref, double... vals) {
        int i = 0;
        for (var tbl : tbl_list) {
            tbl.put(new InterpolatingDouble(ref), new InterpolatingDouble(vals[i]));
            i++;
        }
    }

    /**
     * Gets the interpolated values given a reference value
     * @param ref the reference value which is matched between different table entries in the reference column
     * @return a double array of the interpolated values in the other columns
     */
    public double[] get(double ref) {
        double[] vals = new double[]{};
        int i = 0;
        for (var tbl : tbl_list) {
            vals[i] = tbl.getInterpolated(new InterpolatingDouble(ref)).value;
            i++;
        }

        return vals;
    }

    /**
     * Gets an interpolated value of a given column in the table
     * @param ref the reference value which is matched between different table entries in the reference column
     * @param idx the column number of interest (the first column after the reference column is indexed with 0)
     * @return the interpolated value
     */
    public double get(double ref, int idx) {
        return tbl_list.get(idx).getInterpolated(new InterpolatingDouble(ref)).value;
    }
}
