use core::{
    pin::Pin,
    task::{Context, Poll},
};
use futures::prelude::*;

pub enum Output3<A, B, C>
where
    A: Future,
    B: Future,
    C: Future,
{
    A(A::Output, B, C),
    B(A, B::Output, C),
    C(A, B, C::Output),
}

pub struct Select3<A, B, C>(Option<(A, B, C)>)
where
    A: Future + Unpin,
    B: Future + Unpin,
    C: Future + Unpin;

impl<A, B, C> Select3<A, B, C>
where
    A: Future + Unpin,
    B: Future + Unpin,
    C: Future + Unpin,
{
    pub fn new(a: A, b: B, c: C) -> Self {
        Self(Some((a, b, c)))
    }
}

impl<A, B, C> Future for Select3<A, B, C>
where
    A: Future + Unpin,
    B: Future + Unpin,
    C: Future + Unpin,
{
    type Output = Output3<A, B, C>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let select3 = self.get_mut();
        let (mut a, mut b, mut c) = select3.0.take().expect("cannot poll Select3 twice");
        match Pin::new(&mut a).poll(cx) {
            Poll::Ready(a) => Poll::Ready(Output3::A(a, b, c)),
            Poll::Pending => match Pin::new(&mut b).poll(cx) {
                Poll::Ready(b) => Poll::Ready(Output3::B(a, b, c)),
                Poll::Pending => match Pin::new(&mut c).poll(cx) {
                    Poll::Ready(c) => Poll::Ready(Output3::C(a, b, c)),
                    Poll::Pending => {
                        select3.0 = Some((a, b, c));
                        Poll::Pending
                    }
                },
            },
        }
    }
}
