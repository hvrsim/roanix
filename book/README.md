# Roanix Internals Book

This directory contains *Roanix Internals*, a complete guide to the Roanix kernel.

The book is written in markdown, using [mdBook](https://github.com/rust-lang/mdBook) for generating the online site.

You can also view read the book directly, the table of contents are in [SUMMARY.md](src/SUMMARY.md).

## Building the book

Install the mdbook CLI:

```bash
cargo +stable install mdbook
```

To serve a live version of the book from localhost:

```bash
mdbook serve --open
```

To build the static site from the markdown source in `src/`:

```bash
mdbook build
```

*NOTE: The compiled output will be in `book/`, pass `-d <dir>` to mdBook for a alternative output directory.*