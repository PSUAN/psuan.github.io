---
title: On trait based constructors in Rust
short: "A voice against blanket implementations of constructor traits."
---

# On trait based constructors in Rust

Let's suppose we have some generic enumeration in Rust:

```rust
mod few {
    pub enum Few<A> {
        One(A),
        Two(A, A),
    }
}
```

There are at least three ways to implement instance construction for this enumeration:

1. Direct construction;
2. Blanket constructors;
3. Static constructors;

## Direct construction

In user code we can write the following:

```rust
use few::Few;

let value = Few::One("foo");
```

Which may become a bit repetitive.
To avoid repeating we may `use` specific enumeration variants:

```rust
use few::Few::{One};

let value = One("foo");
```

So we as a user have to import specific definitions and hope that they will not conflict with other types of our module.

That approach is used for `core`'s `Option` enumeration in the Rust standard library.

## Blanket construction

A converter trait is introduced inside the `few` module:

```rust
pub trait IntoOne: Sized {
    fn one(self) -> Few<A>;
}
```

And a blanket implementation is added:

```rust
impl<P> IntoOne for P {
    fn one(self) -> Few<Self> {
        Few::One(self)
    }
}
```

Then in the user code we are to write the following:

```rust
use few::IntoOne as _;

let value = "foo".one();
```

That approach hopes that the instance we call `one` on does not implement a method with the same name.
Otherwise we are to use `IntoOne` directly:

```rust
use few::IntoOne;

let value = IntoOne::one("foo");
```

This approach can be encountered in the [`plotters`](https://crates.io/crates/plotters) crate (for example, the `IntoTextStyle` trait).

## Static constructors

Just add a static function in the `few` module:

```rust
fn one<A>(value: A) -> Few<A> {
    Few::One(value)
}
```

In the user code we are to `use` it or to call it using the full module path:

```rust
let value = few::one("foo");
```

The [`ugly-graphics`](https://crates.io/crates/ugly-graphics) migrated to this approach during the first update.

## Personal opinion

> The static constructor approach is the best one.
> It does not clutter namespace and allows us to use partly-qualified names in our code to reduce name collision risks.
