package magcal

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestQuadricFit(t *testing.T) {
	vals := [][3]float64{
		{7.0, 22.0, 31.0},
		{7.0, 19.0, 28.0},
		{9.0, 23.0, 31.0},
		{9.0, 19.0, 27.0},
		{11.0, 24.0, 29.0},
		{11.0, 20.0, 26.0},
		{8.0, 21.0, 32.0},
		{8.0, 17.0, 29.0},
		{10.0, 22.0, 32.0},
		{10.0, 18.0, 28.0},
		{12.0, 23.0, 31.0},
		{12.0, 19.0, 28.0},
	}

	coeff, err := QuadricFit(vals)
	assert.Nil(t, err)

	assert.True(t, floatEqual(coeff[0], -0.000555, 1e-6))
	assert.True(t, floatEqual(coeff[1], -0.000834, 1e-6))
	assert.True(t, floatEqual(coeff[2], -0.000977, 1e-6))
	assert.True(t, floatEqual(coeff[3], 0.000483, 1e-6))
	assert.True(t, floatEqual(coeff[4], -0.000468, 1e-6))
	assert.True(t, floatEqual(coeff[5], 0.000258, 1e-6))
	assert.True(t, floatEqual(coeff[6], 0.009216, 1e-6))
	assert.True(t, floatEqual(coeff[7], 0.004983, 1e-6))
	assert.True(t, floatEqual(coeff[8], 0.027798, 1e-6))
}

func TestCalMagn(t *testing.T) {
	vals := [][3]float64{
		{2.0, 4.0, -5.0},
		{1.1, 8.3, 9.9},
		{-0.4, -6.6, 3.4},
		{1.5, -6.9, 0.1},
		{0.0, -2.5, -9.8},
		{1.2, 3.8, -0.9},
		{6.1, -4.3, 8.8},
		{-7.6, -0.9, 2.9},
		{5.0, 7.1, 1.0},
		{0.0, -4.7, 0.2},
		{4.3, 0.0, 5.7},
		{-9.0, -2.8, 7.3},
	}

	soft, hard, err := Calibrate(vals, 44.3867)
	assert.Nil(t, err)

	assert.True(t, floatEqual(soft[0], 5.685746, 1e-6))
	assert.True(t, floatEqual(soft[1], 0.091864, 1e-6))
	assert.True(t, floatEqual(soft[2], 0.870620, 1e-6))
	assert.True(t, floatEqual(soft[3], 0.000000, 1e-6))
	assert.True(t, floatEqual(soft[4], 6.375517, 1e-6))
	assert.True(t, floatEqual(soft[5], -0.588939, 1e-6))
	assert.True(t, floatEqual(soft[6], 0.000000, 1e-6))
	assert.True(t, floatEqual(soft[7], 0.000000, 1e-6))
	assert.True(t, floatEqual(soft[8], 2.783505, 1e-6))

	assert.True(t, floatEqual(hard[0], 1.088338, 1e-6))
	assert.True(t, floatEqual(hard[1], -0.900774, 1e-6))
	assert.True(t, floatEqual(hard[2], -7.088266, 1e-6))
}

func floatEqual(a, b, epsilon float64) bool {
	a = a - b
	return a < epsilon && -a < epsilon
}
