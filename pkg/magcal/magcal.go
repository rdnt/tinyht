package magcal

import (
	"errors"

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

	return [9]float64(btmp.RawMatrix().Data), nil
}

// Calibrate calculates the soft iron [K] and the hard iron [b] errors in
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
func Calibrate(m [][3]float64, me ...float64) (K [9]float64, b [3]float64, err error) {
	var me2 = 50.0
	if len(me) > 0 {
		me2 = me[0]
	}

	coeff, err := QuadricFit(m)
	if err != nil {
		return [9]float64{}, [3]float64{}, err
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
	L := mat.NewTriDense(3, mat.Lower, nil)
	G := mat.NewDense(3, 3, nil)
	Ai := mat.NewDense(3, 3, nil)

	var chol mat.Cholesky

	ok := chol.Factorize(A)
	if !ok {
		return [9]float64{}, [3]float64{}, errors.New("matrix is not positive definite")
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

	K = [9]float64(Km.RawMatrix().Data)

	err = Ai.Inverse(A)
	if err != nil {
		return K, [3]float64{}, err
	}

	X0.Mul(Ai, v)

	b = [3]float64(X0.RawMatrix().Data)

	return K, b, nil
}
