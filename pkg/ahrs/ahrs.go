package ahrs

import "math"

// Madgwick instance
type Madgwick struct {
	Beta        float64
	Quaternions [4]float64
}

// NewMadgwick initiates a Madwick struct
func NewMadgwick(beta float64) *Madgwick {
	return &Madgwick{
		Beta:        beta,
		Quaternions: [4]float64{1, 0, 0, 0},
	}
}

// Update9D udpates position using 9D, returning quaternions
func (m *Madgwick) Update9D(gx, gy, gz, ax, ay, az, mx, my, mz, dt float64) {
	var recipNorm float64
	var s0, s1, s2, s3 float64
	var qDot1, qDot2, qDot3, qDot4 float64
	var hx, hy float64
	var _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3 float64

	q0 := m.Quaternions[0]
	q1 := m.Quaternions[1]
	q2 := m.Quaternions[2]
	q3 := m.Quaternions[3]
	beta := m.Beta

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5 * (-q1*gx - q2*gy - q3*gz)
	qDot2 = 0.5 * (q0*gx + q2*gz - q3*gy)
	qDot3 = 0.5 * (q0*gy - q1*gz + q3*gx)
	qDot4 = 0.5 * (q0*gz + q1*gy - q2*gx)

	if !(ax == 0.0 && ay == 0.0 && az == 0.0) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax*ax + ay*ay + az*az)
		ax *= recipNorm
		ay *= recipNorm
		az *= recipNorm

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx*mx + my*my + mz*mz)
		mx *= recipNorm
		my *= recipNorm
		mz *= recipNorm

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0 * q0 * mx
		_2q0my = 2.0 * q0 * my
		_2q0mz = 2.0 * q0 * mz
		_2q1mx = 2.0 * q1 * mx
		_2q0 = 2.0 * q0
		_2q1 = 2.0 * q1
		_2q2 = 2.0 * q2
		_2q3 = 2.0 * q3
		_2q0q2 = 2.0 * q0 * q2
		_2q2q3 = 2.0 * q2 * q3
		q0q0 = q0 * q0
		q0q1 = q0 * q1
		q0q2 = q0 * q2
		q0q3 = q0 * q3
		q1q1 = q1 * q1
		q1q2 = q1 * q2
		q1q3 = q1 * q3
		q2q2 = q2 * q2
		q2q3 = q2 * q3
		q3q3 = q3 * q3

		// Reference direction of Earth's magnetic field
		hx = mx*q0q0 - _2q0my*q3 + _2q0mz*q2 + mx*q1q1 + _2q1*my*q2 + _2q1*mz*q3 - mx*q2q2 - mx*q3q3
		hy = _2q0mx*q3 + my*q0q0 - _2q0mz*q1 + _2q1mx*q2 - my*q1q1 + my*q2q2 + _2q2*mz*q3 - my*q3q3
		_2bx = math.Sqrt(hx*hx + hy*hy)
		_2bz = -_2q0mx*q2 + _2q0my*q1 + mz*q0q0 + _2q1mx*q3 - mz*q1q1 + _2q2*my*q3 - mz*q2q2 + mz*q3q3
		_4bx = 2.0 * _2bx
		_4bz = 2.0 * _2bz

		// Gradient decent algorithm corrective step
		s0 = -_2q2*(2.0*q1q3-_2q0q2-ax) + _2q1*(2.0*q0q1+_2q2q3-ay) - _2bz*q2*(_2bx*(0.5-q2q2-q3q3)+_2bz*(q1q3-q0q2)-mx) + (-_2bx*q3+_2bz*q1)*(_2bx*(q1q2-q0q3)+_2bz*(q0q1+q2q3)-my) + _2bx*q2*(_2bx*(q0q2+q1q3)+_2bz*(0.5-q1q1-q2q2)-mz)
		s1 = _2q3*(2.0*q1q3-_2q0q2-ax) + _2q0*(2.0*q0q1+_2q2q3-ay) - 4.0*q1*(1-2.0*q1q1-2.0*q2q2-az) + _2bz*q3*(_2bx*(0.5-q2q2-q3q3)+_2bz*(q1q3-q0q2)-mx) + (_2bx*q2+_2bz*q0)*(_2bx*(q1q2-q0q3)+_2bz*(q0q1+q2q3)-my) + (_2bx*q3-_4bz*q1)*(_2bx*(q0q2+q1q3)+_2bz*(0.5-q1q1-q2q2)-mz)
		s2 = -_2q0*(2.0*q1q3-_2q0q2-ax) + _2q3*(2.0*q0q1+_2q2q3-ay) - 4.0*q2*(1-2.0*q1q1-2.0*q2q2-az) + (-_4bx*q2-_2bz*q0)*(_2bx*(0.5-q2q2-q3q3)+_2bz*(q1q3-q0q2)-mx) + (_2bx*q1+_2bz*q3)*(_2bx*(q1q2-q0q3)+_2bz*(q0q1+q2q3)-my) + (_2bx*q0-_4bz*q2)*(_2bx*(q0q2+q1q3)+_2bz*(0.5-q1q1-q2q2)-mz)
		s3 = _2q1*(2.0*q1q3-_2q0q2-ax) + _2q2*(2.0*q0q1+_2q2q3-ay) + (-_4bx*q3+_2bz*q1)*(_2bx*(0.5-q2q2-q3q3)+_2bz*(q1q3-q0q2)-mx) + (-_2bx*q0+_2bz*q2)*(_2bx*(q1q2-q0q3)+_2bz*(q0q1+q2q3)-my) + _2bx*q1*(_2bx*(q0q2+q1q3)+_2bz*(0.5-q1q1-q2q2)-mz)
		recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3) // normalise step magnitude
		s0 *= recipNorm
		s1 *= recipNorm
		s2 *= recipNorm
		s3 *= recipNorm

		// Apply feedback step
		qDot1 -= beta * s0
		qDot2 -= beta * s1
		qDot3 -= beta * s2
		qDot4 -= beta * s3
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt
	q1 += qDot2 * dt
	q2 += qDot3 * dt
	q3 += qDot4 * dt

	// Normalise quaternion
	recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
	m.Quaternions[0] = q0 * recipNorm
	m.Quaternions[1] = q1 * recipNorm
	m.Quaternions[2] = q2 * recipNorm
	m.Quaternions[3] = q3 * recipNorm
}

// Update6D updates position using 6D, returning quaternions
func (m *Madgwick) Update6D(gx, gy, gz, ax, ay, az, dt float64) [4]float64 {
	var recipNorm float64
	var s0, s1, s2, s3 float64
	var qDot1, qDot2, qDot3, qDot4 float64
	var _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3 float64

	q0 := m.Quaternions[0]
	q1 := m.Quaternions[1]
	q2 := m.Quaternions[2]
	q3 := m.Quaternions[3]
	beta := m.Beta

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5 * (-q1*gx - q2*gy - q3*gz)
	qDot2 = 0.5 * (q0*gx + q2*gz - q3*gy)
	qDot3 = 0.5 * (q0*gy - q1*gz + q3*gx)
	qDot4 = 0.5 * (q0*gz + q1*gy - q2*gx)

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if !((ax == 0.0) && (ay == 0.0) && (az == 0.0)) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax*ax + ay*ay + az*az)
		ax *= recipNorm
		ay *= recipNorm
		az *= recipNorm

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0 * q0
		_2q1 = 2.0 * q1
		_2q2 = 2.0 * q2
		_2q3 = 2.0 * q3
		_4q0 = 4.0 * q0
		_4q1 = 4.0 * q1
		_4q2 = 4.0 * q2
		_8q1 = 8.0 * q1
		_8q2 = 8.0 * q2
		q0q0 = q0 * q0
		q1q1 = q1 * q1
		q2q2 = q2 * q2
		q3q3 = q3 * q3

		// Gradient decent algorithm corrective step
		s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay
		s1 = _4q1*q3q3 - _2q3*ax + 4.0*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az
		s2 = 4.0*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az
		s3 = 4.0*q1q1*q3 - _2q1*ax + 4.0*q2q2*q3 - _2q2*ay
		recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3) // normalise step magnitude
		s0 *= recipNorm
		s1 *= recipNorm
		s2 *= recipNorm
		s3 *= recipNorm

		// Apply feedback step
		qDot1 -= beta * s0
		qDot2 -= beta * s1
		qDot3 -= beta * s2
		qDot4 -= beta * s3
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt
	q1 += qDot2 * dt
	q2 += qDot3 * dt
	q3 += qDot4 * dt

	// Normalise quaternion
	recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
	m.Quaternions[0] = q0 * recipNorm
	m.Quaternions[1] = q1 * recipNorm
	m.Quaternions[2] = q2 * recipNorm
	m.Quaternions[3] = q3 * recipNorm

	return m.Quaternions
}

func invSqrt(x float64) float64 {
	return 1 / math.Sqrt(x)
}
