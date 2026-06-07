TARGET := thumbv7em-none-eabihf
PROFILE := release
PROJECT_DIR := $(CURDIR)
ELF := target/$(TARGET)/$(PROFILE)/rm_robot
BIN := target/$(TARGET)/$(PROFILE)/rm_robot.bin
CARGO := $(HOME)/.cargo/bin/cargo

.PHONY: all fmt test clippy check build size flash clean

all: build

fmt:
	$(CARGO) fmt --all -- --check

test:
	$(CARGO) test --target x86_64-unknown-linux-gnu --lib

clippy:
	$(CARGO) clippy --target x86_64-unknown-linux-gnu --lib -- -D warnings

check: fmt test clippy build

build:
	cd /tmp && \
		CARGO_TARGET_THUMBV7EM_NONE_EABIHF_RUSTFLAGS="-C link-arg=-Tlink.x" \
		$(CARGO) build --manifest-path $(PROJECT_DIR)/Cargo.toml \
		--target $(TARGET) --release
	arm-none-eabi-objcopy -O binary -S $(ELF) $(BIN)
	arm-none-eabi-size $(ELF)

size:
	arm-none-eabi-size $(ELF)

flash: build
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
		-c "program $(ELF) verify reset exit"

clean:
	$(CARGO) clean
