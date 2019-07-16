build_target := 'thumbv7m-none-eabi'
test_target := 'thumbv7m-linux-eabi'
features := 'stm32l4s7'

# Check with clippy.
clippy:
	cargo clippy --target {{build_target}} --features {{features}} --all

# Generate documentation.
doc:
	cargo doc --target {{build_target}} --features {{features}} --all

# Generate README.md from src/lib.rs.
readme:
	cargo readme -o README.md

# Run tests.
test:
	RUST_TARGET_PATH=$(pwd) CROSS_COMPILE=arm-none-eabi- \
		xargo test --target {{test_target}} --features {{features}} --all
