package magcal

import (
	"testing"

	"github.com/stretchr/testify/require"
)

func TestQuadricFit(t *testing.T) {
	t.Run("coeff", func(t *testing.T) {
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
		require.Nil(t, err)

		require.True(t, floatEqual(coeff[0], -0.000555, 1e-6))
		require.True(t, floatEqual(coeff[1], -0.000834, 1e-6))
		require.True(t, floatEqual(coeff[2], -0.000977, 1e-6))
		require.True(t, floatEqual(coeff[3], 0.000483, 1e-6))
		require.True(t, floatEqual(coeff[4], -0.000468, 1e-6))
		require.True(t, floatEqual(coeff[5], 0.000258, 1e-6))
		require.True(t, floatEqual(coeff[6], 0.009216, 1e-6))
		require.True(t, floatEqual(coeff[7], 0.004983, 1e-6))
		require.True(t, floatEqual(coeff[8], 0.027798, 1e-6))
	})

	t.Run("no err", func(t *testing.T) {
		vals := [][3]float64{
			{13.0, 12.0, 41.0},
			{2.0, 19.0, 21.0},
			{9.0, 23.0, 31.0},
			{6.0, 19.0, 27.0},
			{11.0, 11.0, 29.0},
			{18.0, 20.0, 20.0},
			{8.0, 21.0, 32.0},
			{8.0, 17.0, 29.0},
			{10.0, 22.0, 32.0},
			{13.0, 29.0, 38.0},
			{12.0, 23.0, 39.0},
			{12.0, 12.0, 28.0},
		}

		_, err := QuadricFit(vals)
		require.Nil(t, err)
	})
}

func TestCalibrate(t *testing.T) {
	t.Run("calibrate", func(t *testing.T) {
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

		hard, soft, err := Calibrate(vals, 44.3867)
		require.Nil(t, err)

		require.True(t, floatEqual(soft[0], 5.685746, 1e-6))
		require.True(t, floatEqual(soft[1], 0.091864, 1e-6))
		require.True(t, floatEqual(soft[2], 0.870620, 1e-6))
		require.True(t, floatEqual(soft[3], 0.000000, 1e-6))
		require.True(t, floatEqual(soft[4], 6.375517, 1e-6))
		require.True(t, floatEqual(soft[5], -0.588939, 1e-6))
		require.True(t, floatEqual(soft[6], 0.000000, 1e-6))
		require.True(t, floatEqual(soft[7], 0.000000, 1e-6))
		require.True(t, floatEqual(soft[8], 2.783505, 1e-6))

		require.True(t, floatEqual(hard[0], 1.088338, 1e-6))
		require.True(t, floatEqual(hard[1], -0.900774, 1e-6))
		require.True(t, floatEqual(hard[2], -7.088266, 1e-6))

		t.Run("compare with zscilib cholesky", func(t *testing.T) {
			hard2, soft2, err := calibrateZscilib(vals, 44.3867)
			require.Nil(t, err)

			require.Equal(t, hard, hard2)
			require.Equal(t, soft, soft2)
		})
	})

	t.Run("no err", func(t *testing.T) {
		vals := [][3]float64{
			{12.190000, 28.750000, -63.250000},
			{12.190000, 27.620000, -64.060000},
			{12.560000, 26.870000, -63.630000},
			{11.060000, 26.870000, -62.060000},
			{12.190000, 28.000000, -64.060000},
			{12.190000, 27.620000, -62.500000},
			{11.810000, 27.620000, -62.880000},
			{11.810000, 29.060000, -61.690000},
			{12.560000, 27.250000, -62.060000},
			{12.190000, 28.000000, -62.880000},
			{11.810000, 28.750000, -62.500000},
			{12.560000, 29.440000, -61.690000},
			{11.810000, 29.440000, -62.880000},
			{12.560000, 29.810000, -62.500000},
			{11.810000, 28.750000, -63.250000},
			{11.060000, 28.370000, -63.250000},
			{11.810000, 27.620000, -62.880000},
			{11.440000, 26.870000, -62.500000},
			{12.560000, 26.870000, -63.690000},
			{11.810000, 27.250000, -63.250000},
			{12.560000, 27.620000, -62.500000},
			{13.310000, 26.120000, -62.880000},
			{13.690000, 27.250000, -62.500000},
			{11.440000, 24.310000, -64.440000},
			{11.440000, 24.310000, -64.440000},
			{12.560000, 23.940000, -64.060000},
			{9.250000, 22.060000, -66.810000},
			{7.370000, 19.120000, -68.370000},
			{4.440000, 17.310000, -68.310000},
			{4.440000, 16.940000, -67.190000},
			{2.620000, 15.810000, -68.750000},
			{1.120000, 16.190000, -69.870000},
			{-0.690000, 14.380000, -69.870000},
			{6.690000, 17.690000, -68.370000},
			{8.500000, 21.370000, -66.750000},
			{8.880000, 20.620000, -66.810000},
			{7.750000, 19.120000, -68.000000},
			{7.370000, 21.370000, -67.190000},
			{11.060000, 20.620000, -65.250000},
			{13.310000, 21.750000, -62.880000},
			{15.880000, 22.440000, -62.130000},
			{17.000000, 21.750000, -60.560000},
			{18.810000, 21.000000, -60.130000},
			{17.370000, 19.120000, -60.500000},
			{17.370000, 16.940000, -61.310000},
			{20.310000, 13.630000, -59.000000},
			{22.500000, 12.500000, -57.440000},
			{21.810000, 12.500000, -58.560000},
			{20.690000, 11.000000, -59.380000},
			{21.060000, 14.690000, -59.750000},
			{21.440000, 11.000000, -58.630000},
			{19.940000, 5.870000, -59.750000},
			{21.060000, 8.060000, -60.560000},
			{16.620000, 2.190000, -62.060000},
			{15.130000, -2.620000, -61.690000},
			{19.190000, -5.560000, -58.940000},
			{19.940000, 0.310000, -60.560000},
			{23.620000, -0.370000, -57.810000},
			{26.190000, -5.190000, -52.690000},
			{26.190000, -5.190000, -53.880000},
			{24.750000, -1.870000, -56.250000},
			{21.440000, 2.190000, -59.750000},
			{19.940000, 2.560000, -61.310000},
			{20.690000, 5.120000, -60.500000},
			{18.440000, 5.500000, -62.130000},
			{15.500000, 6.620000, -64.060000},
			{13.690000, 3.310000, -63.250000},
			{12.190000, 3.620000, -64.060000},
			{10.380000, 6.250000, -66.370000},
			{6.310000, 7.000000, -66.810000},
			{4.060000, 4.000000, -66.750000},
			{-0.690000, 3.620000, -67.940000},
			{-4.000000, 4.370000, -68.310000},
			{-8.440000, 4.000000, -68.370000},
			{-13.630000, 4.750000, -68.310000},
			{-15.810000, 2.940000, -68.310000},
			{-18.370000, 2.940000, -68.750000},
			{-21.370000, 1.810000, -68.000000},
			{-25.370000, 4.370000, -65.190000},
			{-30.560000, 2.940000, -62.500000},
			{-32.750000, 3.310000, -60.940000},
			{-35.380000, 2.940000, -59.750000},
			{-39.440000, 3.620000, -57.000000},
			{-42.000000, 4.370000, -55.440000},
			{-45.310000, 4.000000, -55.060000},
			{-46.810000, 4.370000, -51.190000},
			{-49.750000, 1.440000, -46.880000},
			{-50.810000, 4.000000, -46.500000},
			{-50.880000, 2.940000, -45.310000},
			{-52.690000, 3.620000, -41.380000},
			{-55.250000, 3.310000, -41.380000},
			{-54.190000, 4.370000, -37.500000},
			{-56.000000, 3.620000, -35.130000},
			{-56.750000, 4.000000, -30.060000},
			{-58.190000, 3.620000, -26.940000},
			{-57.880000, 4.000000, -22.250000},
			{-58.190000, 5.120000, -17.940000},
			{-57.500000, 5.120000, -15.190000},
			{-54.880000, 8.060000, -12.500000},
			{-56.750000, 5.870000, -9.750000},
			{-54.500000, 7.690000, -9.380000},
			{-53.440000, 6.250000, -6.620000},
			{-53.440000, 7.310000, -3.870000},
			{-51.190000, 6.250000, 0.000000},
			{-49.750000, 9.190000, -0.750000},
			{-49.000000, 9.560000, 3.120000},
			{-48.630000, 8.440000, 3.120000},
			{-48.250000, 10.690000, 5.060000},
			{-46.060000, 11.380000, 5.440000},
			{-45.690000, 11.000000, 6.620000},
			{-45.310000, 11.380000, 7.810000},
			{-42.000000, 11.380000, 10.130000},
			{-42.000000, 9.560000, 10.500000},
			{-39.440000, 11.000000, 12.060000},
			{-37.940000, 10.690000, 14.060000},
			{-37.190000, 9.190000, 12.880000},
			{-35.750000, 7.690000, 14.440000},
			{-35.750000, 8.810000, 14.810000},
			{-37.190000, 6.620000, 14.440000},
			{-36.810000, 7.690000, 14.810000},
			{-36.440000, 7.310000, 14.060000},
			{-36.130000, 8.440000, 14.810000},
			{-36.810000, 8.060000, 14.060000},
			{-39.060000, 8.810000, 13.250000},
			{-37.940000, 8.440000, 12.500000},
			{-39.060000, 8.440000, 12.500000},
			{-39.810000, 9.190000, 12.060000},
			{-39.810000, 7.310000, 12.060000},
			{-41.630000, 9.560000, 10.940000},
			{-42.000000, 7.310000, 9.750000},
			{-43.500000, 8.060000, 8.190000},
			{-47.190000, 8.810000, 4.690000},
			{-50.130000, 8.060000, 1.940000},
			{-50.130000, 8.060000, 1.940000},
			{-51.940000, 8.810000, -1.940000},
			{-55.250000, 9.190000, -5.810000},
			{-56.380000, 8.810000, -11.690000},
			{-58.190000, 5.500000, -18.310000},
			{-58.940000, 6.250000, -21.870000},
			{-57.880000, 6.620000, -27.310000},
			{-57.880000, 7.000000, -35.130000},
			{-55.250000, 5.870000, -42.190000},
			{-49.380000, 5.870000, -50.380000},
			{-43.500000, 4.000000, -55.880000},
			{-40.880000, 4.750000, -59.000000},
			{-33.500000, 4.370000, -62.130000},
			{-27.250000, 4.000000, -65.250000},
			{-21.000000, 2.940000, -66.440000},
			{-13.630000, 5.500000, -68.000000},
			{-7.000000, 7.310000, -68.750000},
			{2.620000, 6.250000, -66.000000},
			{7.750000, 7.000000, -65.250000},
			{9.630000, 5.870000, -64.440000},
			{16.620000, 5.120000, -62.130000},
			{20.310000, 4.750000, -59.000000},
			{23.620000, 5.500000, -56.250000},
			{25.500000, 5.870000, -54.690000},
			{29.870000, 6.620000, -51.190000},
			{29.870000, 8.060000, -50.810000},
			{31.750000, 12.880000, -49.190000},
			{32.130000, 10.690000, -48.810000},
			{28.810000, 9.560000, -51.560000},
			{29.870000, 6.620000, -51.190000},
			{30.250000, 4.750000, -49.630000},
			{29.500000, 3.310000, -50.810000},
			{29.500000, 1.810000, -51.190000},
			{28.060000, 2.190000, -50.000000},
			{27.690000, -0.370000, -52.000000},
			{26.560000, -3.000000, -53.130000},
			{25.120000, -3.690000, -52.380000},
			{23.620000, -4.440000, -53.940000},
			{22.120000, -5.940000, -53.560000},
			{22.870000, -8.130000, -52.380000},
			{23.250000, -5.190000, -51.190000},
			{25.500000, -4.060000, -51.190000},
			{29.870000, -3.370000, -46.130000},
			{30.250000, -3.690000, -46.130000},
			{29.500000, -3.690000, -46.500000},
			{29.500000, -3.000000, -45.690000},
			{31.750000, -5.190000, -43.000000},
			{31.750000, -5.560000, -41.810000},
			{32.500000, -1.500000, -41.000000},
			{33.190000, -2.250000, -39.880000},
			{31.750000, -4.440000, -38.690000},
			{34.690000, -4.440000, -37.880000},
			{35.060000, -5.190000, -35.940000},
			{32.880000, -6.690000, -34.000000},
			{33.940000, -4.440000, -32.810000},
			{36.190000, -3.690000, -33.190000},
			{35.810000, -2.250000, -33.190000},
			{35.810000, -2.620000, -30.500000},
			{33.560000, -1.500000, -30.870000},
			{38.000000, -1.120000, -28.500000},
			{38.000000, 2.940000, -19.120000},
			{36.190000, 3.310000, -12.060000},
			{35.060000, 2.560000, -8.560000},
			{32.130000, 3.310000, -3.120000},
			{31.000000, 5.120000, 2.690000},
			{26.560000, 3.620000, 8.940000},
			{24.370000, 5.870000, 11.690000},
			{20.690000, 6.620000, 15.190000},
			{13.310000, 8.060000, 17.560000},
			{7.060000, 8.810000, 22.620000},
			{0.370000, 7.310000, 22.250000},
			{-5.120000, 7.690000, 23.060000},
			{-10.310000, 8.810000, 23.810000},
			{-12.500000, 11.750000, 23.440000},
			{-14.000000, 12.130000, 23.440000},
			{-18.750000, 8.440000, 23.060000},
			{-20.620000, 6.250000, 22.620000},
			{-23.940000, 5.500000, 21.870000},
			{-27.620000, 4.750000, 19.500000},
			{-31.690000, 7.000000, 19.120000},
			{-34.250000, 6.620000, 17.940000},
			{-36.810000, 7.310000, 16.000000},
			{-39.060000, 11.000000, 15.630000},
			{-41.250000, 10.690000, 15.190000},
			{-41.250000, 9.940000, 13.630000},
			{-40.500000, 10.310000, 13.630000},
			{-39.440000, 10.690000, 14.060000},
			{-39.810000, 10.310000, 12.880000},
			{-41.630000, 9.560000, 13.250000},
			{-41.250000, 10.310000, 13.630000},
			{-39.810000, 9.560000, 14.440000},
			{-38.690000, 8.060000, 15.630000},
			{-36.440000, 8.440000, 16.810000},
			{-30.940000, 9.190000, 19.120000},
			{-29.810000, 8.810000, 18.370000},
			{-25.060000, 7.690000, 20.310000},
			{-21.750000, 8.060000, 22.250000},
			{-12.880000, 8.060000, 22.620000},
			{-3.310000, 6.250000, 22.620000},
			{5.190000, 3.620000, 20.690000},
			{11.440000, 4.370000, 18.370000},
			{23.620000, 0.310000, 8.560000},
			{27.310000, -0.370000, 4.250000},
			{31.750000, -1.870000, -0.370000},
			{32.880000, -3.690000, -4.250000},
			{34.310000, -4.060000, -14.810000},
			{33.940000, -4.440000, -26.190000},
			{33.190000, -6.690000, -38.310000},
			{31.370000, -7.060000, -43.750000},
			{26.560000, -8.130000, -48.440000},
			{23.250000, -7.060000, -52.000000},
			{19.560000, -7.750000, -55.130000},
			{17.000000, -7.060000, -57.810000},
			{15.500000, -7.370000, -58.630000},
			{12.940000, -8.500000, -60.190000},
			{12.560000, -6.310000, -60.630000},
			{12.940000, -4.440000, -60.190000},
			{14.060000, -0.750000, -59.810000},
			{14.440000, -1.500000, -59.440000},
			{15.500000, 0.310000, -61.380000},
			{16.250000, -0.370000, -60.190000},
			{17.750000, 1.440000, -59.810000},
			{16.250000, 1.440000, -60.190000},
			{18.810000, 1.060000, -60.630000},
			{17.750000, 0.690000, -60.630000},
			{18.120000, 0.310000, -58.630000},
			{18.810000, 1.810000, -59.440000},
			{17.000000, 1.440000, -59.810000},
			{15.880000, 4.370000, -60.190000},
			{16.620000, 9.940000, -60.630000},
			{15.880000, 14.000000, -61.380000},
			{15.500000, 16.940000, -59.810000},
			{18.810000, 22.060000, -59.000000},
			{17.370000, 26.500000, -58.250000},
			{17.750000, 33.130000, -54.690000},
			{18.440000, 36.130000, -52.000000},
			{15.130000, 39.060000, -49.250000},
			{16.620000, 43.500000, -46.880000},
			{16.250000, 46.060000, -40.250000},
			{18.440000, 46.440000, -38.690000},
			{20.690000, 45.310000, -31.620000},
			{17.750000, 49.750000, -26.940000},
			{19.190000, 49.380000, -21.500000},
			{23.250000, 48.250000, -17.940000},
			{21.060000, 47.190000, -11.690000},
			{20.690000, 47.880000, -7.810000},
			{18.810000, 47.500000, -5.810000},
			{18.120000, 46.060000, -3.500000},
			{17.750000, 42.380000, 0.750000},
			{19.190000, 39.440000, 5.060000},
			{18.810000, 34.630000, 6.250000},
			{18.120000, 32.810000, 10.130000},
			{19.560000, 30.940000, 10.130000},
			{22.870000, 29.120000, 8.190000},
			{22.120000, 27.250000, 10.130000},
			{22.120000, 26.870000, 10.940000},
			{23.250000, 23.190000, 10.940000},
			{22.500000, 23.560000, 12.130000},
			{21.440000, 25.060000, 12.060000},
			{22.500000, 20.620000, 12.500000},
			{21.810000, 19.870000, 13.250000},
			{22.870000, 21.750000, 10.130000},
			{22.870000, 21.000000, 10.940000},
			{22.870000, 20.620000, 10.940000},
			{23.620000, 18.060000, 12.060000},
			{20.690000, 16.190000, 14.810000},
			{19.940000, 16.190000, 16.000000},
			{19.940000, 15.060000, 15.250000},
			{20.310000, 16.940000, 14.440000},
			{22.120000, 20.620000, 10.940000},
			{23.620000, 22.810000, 8.940000},
			{25.120000, 27.250000, 5.060000},
			{26.940000, 35.380000, 1.120000},
			{24.750000, 41.250000, -5.060000},
			{25.500000, 44.190000, -12.060000},
			{23.250000, 45.690000, -19.940000},
			{24.750000, 47.190000, -24.190000},
			{23.620000, 45.690000, -29.310000},
			{23.250000, 44.560000, -37.500000},
			{23.620000, 39.810000, -44.940000},
			{22.870000, 34.250000, -50.810000},
			{23.620000, 27.620000, -55.880000},
			{20.690000, 17.690000, -59.380000},
			{22.870000, 7.310000, -60.560000},
			{21.060000, 1.810000, -60.630000},
			{19.560000, -3.000000, -59.000000},
			{17.750000, -6.690000, -59.810000},
			{17.750000, -6.690000, -61.750000},
			{17.750000, -8.880000, -59.440000},
			{18.120000, -10.380000, -57.500000},
			{17.750000, -14.750000, -57.060000},
			{17.750000, -17.370000, -53.940000},
			{17.370000, -19.190000, -52.380000},
			{17.000000, -24.000000, -48.440000},
			{16.250000, -27.690000, -43.810000},
			{15.130000, -29.190000, -41.810000},
			{16.250000, -31.750000, -37.130000},
			{16.620000, -32.500000, -32.440000},
			{16.250000, -33.940000, -31.690000},
			{17.370000, -34.690000, -24.620000},
			{18.440000, -32.500000, -19.500000},
			{16.620000, -32.880000, -14.060000},
			{15.880000, -33.190000, -9.750000},
			{15.500000, -29.190000, -1.560000},
			{14.060000, -26.190000, 3.870000},
			{12.190000, -22.500000, 7.810000},
			{12.560000, -22.120000, 7.000000},
			{15.130000, -18.440000, 11.690000},
			{15.130000, -16.620000, 10.560000},
			{15.130000, -15.880000, 11.310000},
			{14.440000, -13.310000, 13.250000},
			{15.500000, -12.560000, 14.060000},
			{15.130000, -8.500000, 14.810000},
			{14.750000, -10.000000, 14.810000},
			{17.750000, -8.880000, 13.250000},
			{19.190000, -11.060000, 12.880000},
			{21.060000, -9.250000, 11.690000},
			{21.440000, -8.880000, 11.690000},
			{20.310000, -6.690000, 12.880000},
			{20.690000, -7.060000, 13.250000},
			{22.120000, -8.500000, 11.310000},
			{22.120000, -8.500000, 11.310000},
			{18.440000, -11.440000, 12.500000},
			{18.810000, -9.630000, 12.880000},
			{18.440000, -7.750000, 14.440000},
			{16.620000, -7.750000, 15.250000},
			{17.750000, -7.060000, 15.250000},
			{17.750000, -5.560000, 14.810000},
			{15.500000, -2.620000, 16.370000},
			{17.750000, -1.870000, 17.190000},
			{16.620000, -1.120000, 15.630000},
			{17.370000, 1.060000, 16.000000},
			{15.880000, 1.440000, 16.000000},
			{16.620000, 3.310000, 17.190000},
			{18.120000, 2.190000, 18.370000},
			{17.750000, 1.810000, 17.190000},
			{17.750000, -1.500000, 17.560000},
			{17.000000, -3.690000, 17.560000},
			{14.750000, -8.130000, 17.190000},
			{11.440000, -14.060000, 14.440000},
			{10.000000, -20.310000, 10.940000},
			{11.440000, -24.750000, 6.620000},
			{13.690000, -27.690000, 1.940000},
			{13.310000, -31.750000, -6.250000},
			{9.630000, -34.690000, -12.130000},
			{7.060000, -35.810000, -16.810000},
			{5.940000, -37.630000, -20.690000},
			{5.560000, -37.630000, -26.940000},
			{6.310000, -36.560000, -31.250000},
			{8.130000, -34.310000, -37.560000},
			{10.750000, -30.620000, -42.250000},
			{12.190000, -25.870000, -48.130000},
			{12.190000, -21.060000, -51.630000},
			{14.750000, -18.810000, -52.810000},
			{14.060000, -13.690000, -55.560000},
			{17.750000, -11.810000, -55.560000},
			{17.000000, -7.060000, -57.500000},
			{14.750000, -5.940000, -59.880000},
			{17.000000, -5.560000, -59.880000},
			{17.370000, -4.810000, -57.880000},
			{18.120000, -4.440000, -58.690000},
			{15.500000, -3.690000, -59.440000},
			{16.620000, -5.560000, -59.810000},
			{15.500000, -7.060000, -58.690000},
			{15.880000, -6.690000, -57.880000},
			{17.750000, -8.500000, -59.060000},
			{17.370000, -8.880000, -59.060000},
			{17.750000, -9.630000, -58.690000},
			{21.440000, -7.370000, -57.130000},
			{21.060000, -4.810000, -58.310000},
			{20.310000, -4.440000, -59.440000},
			{19.560000, -1.870000, -60.250000},
			{15.500000, -4.440000, -61.060000},
			{15.500000, -4.060000, -61.060000},
			{15.880000, -3.690000, -61.810000},
			{16.250000, -4.060000, -62.630000},
			{14.440000, -5.190000, -60.250000},
			{14.060000, -8.880000, -59.500000},
			{13.690000, -8.880000, -61.060000},
			{13.690000, -8.130000, -60.630000},
			{12.940000, -9.250000, -61.000000},
			{13.310000, -9.630000, -60.630000},
			{9.250000, -12.940000, -61.000000},
			{7.060000, -17.750000, -61.440000},
			{2.620000, -20.310000, -60.630000},
			{-4.000000, -21.060000, -62.560000},
			{-9.560000, -21.440000, -62.560000},
			{-16.940000, -18.440000, -63.380000},
			{-23.560000, -17.000000, -62.560000},
			{-28.000000, -15.500000, -61.000000},
			{-31.310000, -11.060000, -61.380000},
			{-33.880000, -8.500000, -61.810000},
			{-32.810000, -5.560000, -62.940000},
			{-33.130000, -2.620000, -62.560000},
			{-34.250000, -3.000000, -62.190000},
			{-34.630000, -3.370000, -61.000000},
			{-33.130000, 0.000000, -61.810000},
			{-33.500000, 1.810000, -61.000000},
			{-33.500000, 5.500000, -62.560000},
			{-31.310000, 5.500000, -62.940000},
			{-30.560000, 6.250000, -63.750000},
			{-30.190000, 6.620000, -64.500000},
			{-30.190000, 7.000000, -63.380000},
			{-30.560000, 7.690000, -63.750000},
			{-30.560000, 9.190000, -64.500000},
			{-31.310000, 8.440000, -64.940000},
			{-30.560000, 8.810000, -64.120000},
			{-29.120000, 7.690000, -65.310000},
			{-30.940000, 9.560000, -64.120000},
			{-31.310000, 8.440000, -63.750000},
			{-30.190000, 8.810000, -63.750000},
			{-32.810000, 7.370000, -63.000000},
			{-33.500000, 2.940000, -62.940000},
			{-34.250000, -0.750000, -61.810000},
			{-34.250000, -4.440000, -61.380000},
			{-32.060000, -4.440000, -62.630000},
			{-27.620000, -7.060000, -64.120000},
			{-25.750000, -8.500000, -62.560000},
			{-25.060000, -12.560000, -61.810000},
			{-23.190000, -16.250000, -61.000000},
			{-18.750000, -19.190000, -60.250000},
			{-15.440000, -19.940000, -60.630000},
			{-11.380000, -22.500000, -59.500000},
			{-8.810000, -22.500000, -59.880000},
			{-6.250000, -22.870000, -59.440000},
			{-0.690000, -24.370000, -58.690000},
			{1.870000, -23.250000, -59.880000},
			{6.690000, -20.690000, -59.060000},
			{12.190000, -18.810000, -60.250000},
			{14.750000, -16.620000, -59.880000},
			{17.750000, -15.500000, -60.250000},
			{20.310000, -7.060000, -59.880000},
			{19.190000, -5.560000, -60.250000},
			{15.500000, -7.440000, -61.810000},
			{15.130000, -5.190000, -62.560000},
			{17.750000, -5.190000, -61.060000},
			{18.810000, -5.190000, -61.440000},
			{17.750000, 3.310000, -64.190000},
			{21.440000, 10.310000, -63.000000},
			{18.440000, 15.810000, -64.560000},
			{16.250000, 19.500000, -64.190000},
			{17.000000, 22.060000, -62.630000},
			{14.440000, 25.440000, -62.630000},
			{10.750000, 30.190000, -62.630000},
			{8.500000, 30.940000, -62.190000},
			{4.060000, 32.440000, -62.630000},
			{2.620000, 33.500000, -63.750000},
			{0.370000, 31.310000, -63.380000},
			{-2.940000, 31.310000, -63.380000},
			{-7.310000, 33.130000, -64.190000},
			{-11.000000, 32.810000, -63.000000},
			{-12.880000, 32.440000, -62.250000},
			{-17.310000, 34.630000, -61.810000},
			{-22.440000, 32.440000, -61.810000},
			{-27.620000, 28.750000, -62.190000},
			{-29.810000, 25.440000, -61.810000},
			{-31.310000, 25.750000, -61.440000},
		}

		_, _, err := Calibrate(vals, 44.3867)
		require.Nil(t, err)
	})
}

func floatEqual(a, b, epsilon float64) bool {
	a = a - b
	return a < epsilon && -a < epsilon
}
