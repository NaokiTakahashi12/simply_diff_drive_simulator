DOCKERFILE := Dockerfile
PROJECT := simply-diff-drive-simulator
ORIGIN := $(shell git remote get-url origin | sed -e 's/^.*@//g')
REVISION := $(shell git rev-parse --short HEAD)
DOCKERFILES := $(sort $(wildcard */$(DOCKERFILE)))
USERNAME := naokitakahashi12

SDDS_FOXY_DIR := foxy
SDDS_FOXY_TAG := ros2-foxy

define dockerbuild
	@docker build \
		--file $1 \
		--build-arg GIT_REVISION=$(REVISION) \
		--build-arg GIT_ORIGIN=$(ORIGIN) \
		--tag $2 \
	$3
endef

.PHONY: all
all: help

.PHONY: build
build: \
	$(SDDS_FOXY_TAG)

$(SDDS_FOXY_TAG): $(SDDS_FOXY_DIR)/$(DOCKERFILE)
	$(eval BUILD_DIR := "$(SDDS_FOXY_DIR)")
	$(eval DOCKERIMAGE := "$(USERNAME)/$(PROJECT):$@")
	$(call dockerbuild, $<, $(DOCKERIMAGE), $(BUILD_DIR))
.PHONY: clean
clean:
	@docker image rm $(USERNAME)/$(PROJECT):$(SDDS_FOXY_TAG)

.PHONY: pull
pull:
	@docker image pull $(USERNAME)/$(PROJECT):$(SDDS_FOXY_TAG)

.PHONY: help
help:
	@echo ""
	@echo " make <command> [option]"
	@echo ""
	@echo " Support commands"
	@echo " help  : print support commands"
	@echo " build : building docker images"
	@echo " clean : clean docker images"
	@echo " pull  : pulling docker images from registry"
	@echo ""
	@echo " Options"
	@echo " See the 'make --help'"
	@echo ""

