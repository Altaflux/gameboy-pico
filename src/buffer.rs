use alloc::vec::Vec;

/// Defines methods that would be expected on a queue data structure
pub trait IsQueue<T: Clone> {
    /// Adds a new value to a queue
    ///
    /// # Parameters
    /// - `val`: Value to add to the queue
    ///
    /// # Returns
    /// - `Ok(_)`: If the element add was successful.
    ///     - `Some(T)`: If adding an element resulted in the removal of an
    ///         existing one (in the case of a circular buffer, for instance)
    ///     - `None`: Adding an element did not return any value
    /// - `Error`: If the element add was unsuccessful
    ///
    /// # Errors
    /// Attempting to add an element to a full queue that does not allow for
    /// overflow will return an error.
    fn add(&mut self, val: T) -> Result<Option<T>, &str>;

    /// Removes an element from the queue and returns it
    ///
    /// For queues with default values, removing an element will add a new
    /// default value into the queue.
    ///
    /// # Returns
    /// - `Ok(T)`: The oldest element in the queue
    /// - `Error`
    ///
    /// # Errors
    /// Returns an error if an attempt is made to remove an element from
    /// an empty queue
    fn remove(&mut self) -> Result<T, &str>;

    /// Peek at the head of the queue
    ///
    /// # Returns
    /// - `Ok(T)`: The next element scheduled for removal from the queue
    /// - `Error`
    ///
    /// # Errors
    /// Returns an error if an attempt is made to peek into an empty queue
    fn peek(&self) -> Result<T, &str>;

    /// Gets the size of the queue
    ///
    /// # Returns
    /// The number of elements in the queue. Note, this _includes_ default
    /// values when specified, which means that the `size` of a queue with
    /// default values should always be equal to its `capacity`
    fn size(&self) -> usize;
}

#[derive(Debug)]
pub struct Buffer<T: Clone> {
    queue: Vec<T>,
    capacity: usize,
}

impl<T: Clone> Buffer<T> {
    pub fn new(capacity: usize) -> Buffer<T> {
        Buffer {
            queue: alloc::vec![],
            capacity,
        }
    }

    pub fn capacity(&self) -> usize {
        self.capacity
    }
}

impl<T: Clone> IsQueue<T> for Buffer<T> {
    /// Adds an element to a buffer
    ///
    /// # Parameters
    /// - `val`: Value to add to the buffer
    ///
    /// # Returns
    /// - `Ok(None)`: Element addition was successful
    /// - `Error`
    ///
    /// # Errors
    /// Returns an error if an attempt is made to add an element to a full
    /// buffer
    ///
    /// # Examples
    ///
    /// ```
    /// use queues::*;
    ///
    /// let mut buf: Buffer<isize> = Buffer::new(3);
    /// assert_eq!(buf.add(42), Ok(None));
    /// ```
    fn add(&mut self, val: T) -> Result<Option<T>, &str> {
        if self.queue.len() < self.capacity {
            self.queue.push(val);
            Ok(None)
        } else {
            Err("The buffer is full")
        }
    }

    /// Removes an element from the buffer and returns it.
    ///
    /// # Returns
    /// - `Ok(T)`: The oldest element in the buffer
    /// - `Error`
    ///
    /// # Errors
    /// Returns an error if an attempt is made to remove an element from
    /// an empty buffer
    ///
    /// # Examples
    ///
    /// ```
    /// # use queues::*;
    /// let mut buf: Buffer<isize> = Buffer::new(3);
    /// buf.add(42);
    /// assert_eq!(buf.remove(), Ok(42));
    /// assert_eq!(buf.size(), 0);
    /// ```
    fn remove(&mut self) -> Result<T, &str> {
        if self.queue.len() > 0 {
            Ok(self.queue.remove(0usize))
        } else {
            Err("The buffer is empty")
        }
    }

    /// Peek at the head of the buffer
    ///
    /// # Returns
    /// - `Ok(T)`: The next element scheduled for removal from the buffer
    /// - `Error`
    ///
    /// # Errors
    /// Returns an error if an attempt is made to peek into an empty buffer
    ///
    /// # Examples
    ///
    /// ```
    /// # use queues::*;
    /// let mut buf: Buffer<isize> = Buffer::new(3);
    /// buf.add(42);
    /// assert_eq!(buf.peek(), Ok(42));
    /// ```
    fn peek(&self) -> Result<T, &str> {
        match self.queue.first() {
            Some(val) => Ok(val.clone()),
            None => Err("The buffer is empty"),
        }
    }

    /// Gets the size of the buffer
    ///
    /// # Returns
    /// The number of elements in the buffer.
    ///
    /// # Examples
    ///
    /// ```
    /// # use queues::*;
    /// let mut buf: Buffer<isize> = Buffer::new(3);
    /// assert_eq!(buf.size(), 0);
    /// buf.add(42);
    /// assert_eq!(buf.size(), 1);
    /// ```
    fn size(&self) -> usize {
        self.queue.len()
    }
}
