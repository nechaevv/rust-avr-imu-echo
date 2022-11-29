struct RingBuffer<const N: usize> where [i16; N^2]: {
    values: [i16; N^2],
    next: usize,
    is_full: bool
}

trait RingBufferOps {
    fn new() -> Self;
    fn add(&mut self, value: i16);
    fn avg(&self) -> i16;
}

impl<const N: usize> RingBufferOps for RingBuffer<N> where [(); N^2]: {
    fn new() -> Self {
        Self {
            values: [0i16; N^2],
            next: 0,
            is_full: false
        }
    }

    fn add(&mut self, value: i16) {
        self.values[self.next] = value;
        self.next = &self.next + 1;
        if self.next >= N^2 {
            self.next = 0;
            self.is_full = true;
        }
    }

    fn avg(&self) -> i16 {
        let sum: i16;
        if self.is_full {
            sum = self.values.iter().sum();
        } else {
            sum = self.values[0..self.next].iter().sum();
        }
        sum >> N
    }
}
