package magcal

import (
	"errors"
	"math"

	"gonum.org/v1/gonum/mat"
)

// QuadricFit uses the least squares fitting method to compute the coefficients
// of a quadric surface given a set of tridimensional points [m].
//
// A quadric is a 3D surface that is defined by the equation:
// Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1.
// Spheres and ellipsoids are special cases of quadrics.
// This function takes a set of points (x,y,z) and returns the coefficients
// (A, B, C, D, E, F, G, H, I) of the quadric surface that best fit the given
// points.
//
// This function is a port of zsl_sta_quad_fit from the Zephyr zscilib package.
func QuadricFit(m [][3]float64) ([9]float64, error) {
	x := mat.NewDense(len(m), 9, nil)
	xv := make([]float64, 9)
	y := mat.NewDense(len(m), 1, nil)

	for i, v := range m {
		xv[0] = v[0] * v[0]
		xv[1] = v[1] * v[1]
		xv[2] = v[2] * v[2]
		xv[3] = 2.0 * v[0] * v[1]
		xv[4] = 2.0 * v[0] * v[2]
		xv[5] = 2.0 * v[1] * v[2]
		xv[6] = 2.0 * v[0]
		xv[7] = 2.0 * v[1]
		xv[8] = 2.0 * v[2]

		x.SetRow(i, xv)
		y.SetRow(i, []float64{1.0})
	}

	xtx := mat.NewDense(9, 9, nil)
	inv := mat.NewDense(9, 9, nil)
	xtmp := mat.NewDense(9, len(m), nil)
	btmp := mat.NewDense(9, 1, nil)

	xt := x.T()

	xtx.Mul(xt, x)

	err := inv.Inverse(xtx)
	if err != nil {
		return [9]float64{}, err
	}

	xtmp.Mul(inv, xt)
	btmp.Mul(xtmp, y)

	data := [9]float64{}
	mat.Col(data[:], 0, btmp)

	return data, nil
}

// Calibrate calculates the hard iron [b] and the soft iron [K] errors in
// the given magnetometer data [m], using the least squares method for
// ellipsoid fitting.
//
// [me] is the module of the Earth's magnetic field, and is dependent on location
// and date.
// You can calculate it from this page:
// https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?#igrfwmm
// It will be displayed as the "Total Field", and should be converted from
// nT to uT, so for a given value of 44387.0 nT, we could use 44.387 for [me].
// If unknown, a sensible default of 50.0 is used.
//
// This function is a port of zsl_fus_cal_magn_fast from the Zephyr zscilib package.
func Calibrate(m [][3]float64, me ...float64) (hard [3]float64, soft [9]float64, err error) {
	var me2 = 50.0
	if len(me) > 0 {
		me2 = me[0]
	}

	coeff, err := QuadricFit(m)
	if err != nil {
		return [3]float64{}, [9]float64{}, err
	}

	A := mat.NewSymDense(3, []float64{
		0: coeff[0],
		1: coeff[3],
		2: coeff[4],
		3: coeff[3],
		4: coeff[1],
		5: coeff[5],
		6: coeff[4],
		7: coeff[5],
		8: coeff[2],
	})

	v := mat.NewDense(3, 1, []float64{
		0: coeff[6],
		1: coeff[7],
		2: coeff[8],
	})

	X0 := mat.NewDense(3, 1, nil)
	Ai := mat.NewDense(3, 3, nil)

	err = Ai.Inverse(A)
	if err != nil {
		return [3]float64{}, [9]float64{}, err
	}

	X0.Mul(Ai, v)

	hard = [3]float64(X0.RawMatrix().Data)

	L := mat.NewTriDense(3, mat.Lower, nil)
	G := mat.NewDense(3, 3, nil)

	var chol mat.Cholesky

	ok := chol.Factorize(A)
	if !ok {
		return hard, [9]float64{}, errors.New("matrix is not positive definite")
	}

	chol.LTo(L)

	mul := mat.NewDense(3, 3, []float64{
		0: me2,
		1: me2,
		2: me2,
		3: me2,
		4: me2,
		5: me2,
		6: me2,
		7: me2,
		8: me2,
	})

	G.MulElem(L.T(), mul)

	Km := mat.NewDense(3, 3, nil)
	Km.CloneFrom(G)

	soft = [9]float64(Km.RawMatrix().Data)

	return hard, soft, nil
}

func calibrateZscilib(m [][3]float64, me ...float64) (hard [3]float64, soft [9]float64, err error) {
	var me2 = 50.0
	if len(me) > 0 {
		me2 = me[0]
	}

	coeff, err := QuadricFit(m)
	if err != nil {
		return [3]float64{}, [9]float64{}, err
	}

	A := mat.NewSymDense(3, []float64{
		0: coeff[0],
		1: coeff[3],
		2: coeff[4],
		3: coeff[3],
		4: coeff[1],
		5: coeff[5],
		6: coeff[4],
		7: coeff[5],
		8: coeff[2],
	})

	v := mat.NewDense(3, 1, []float64{
		0: coeff[6],
		1: coeff[7],
		2: coeff[8],
	})

	X0 := mat.NewDense(3, 1, nil)
	Ai := mat.NewDense(3, 3, nil)

	err = Ai.Inverse(A)
	if err != nil {
		return [3]float64{}, soft, err
	}

	X0.Mul(Ai, v)

	hard = [3]float64(X0.RawMatrix().Data)

	L := mat.NewTriDense(3, mat.Lower, nil)
	G := mat.NewDense(3, 3, nil)

	input := [3][3]float64{}
	mat.Row(input[0][:], 0, A)
	mat.Row(input[1][:], 1, A)
	mat.Row(input[2][:], 2, A)
	dia := cholesky(input)

	L.SetTri(0, 0, dia[0][0])
	L.SetTri(1, 0, dia[1][0])
	L.SetTri(1, 1, dia[1][1])
	L.SetTri(2, 0, dia[2][0])
	L.SetTri(2, 1, dia[2][1])
	L.SetTri(2, 2, dia[2][2])

	mul := mat.NewDense(3, 3, []float64{
		0: me2,
		1: me2,
		2: me2,
		3: me2,
		4: me2,
		5: me2,
		6: me2,
		7: me2,
		8: me2,
	})

	G.MulElem(L.T(), mul)

	Km := mat.NewDense(3, 3, nil)
	Km.CloneFrom(G)

	soft = [9]float64(Km.RawMatrix().Data)

	return hard, soft, nil
}

func cholesky(input [3][3]float64) [3][3]float64 {
	var sum, x, y float64
	cols := 3

	lower := [3][3]float64{}

	for j := 0; j < cols; j++ {
		sum = 0
		for k := 0; k < j; k++ {
			x = lower[j][k]
			sum += x * x
		}
		x = input[j][j]
		lower[j][j] = math.Sqrt(x - sum)

		for i := j + 1; i < cols; i++ {
			sum = 0
			for k := 0; k < j; k++ {
				x = lower[j][k]
				y = lower[i][k]
				sum += y * x
			}
			x = lower[j][j]
			y = input[i][j]
			lower[i][j] = (y - sum) / x
		}

	}

	return lower
}
