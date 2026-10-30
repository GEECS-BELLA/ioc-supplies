ARG IMAGE_EXT

ARG REGISTRY=ghcr.io/epics-containers
# Built on the StreamDevice developer image (ioc-streamdevice 2.8.26ec6): EPICS
# base, asyn, StreamDevice, calc, sequencer, sscan, busy, autosave, std,
# iocStats and pvlogging already built, as the upstream runtime geecs-epics'
# plain supplies run. ibek-support is pinned to the commit that image was
# built with (8545fa2). Our emqComposite (an SNL program) is added: the
# upstream runtime carries libseq but no compiler, and an SNL program must be
# linked into the IOC (README "Why ours").
ARG RUNTIME=${REGISTRY}/epics-base${IMAGE_EXT}-runtime:7.0.10ec2
ARG DEVELOPER=${REGISTRY}/ioc-streamdevice${IMAGE_EXT}-developer:2.8.26ec6

##### build stage ##############################################################
FROM  ${DEVELOPER} AS developer

# The devcontainer mounts the project root to /epics/generic-source
# Using the same location here makes devcontainer/runtime differences transparent.
ENV SOURCE_FOLDER=/epics/generic-source
# (The template's `ln -s ${SOURCE_FOLDER}/ioc ${IOC}` is omitted: the
# StreamDevice developer image already made that link, as ioc-motion's base did.)

# get the current versions of pvi and ibek
COPY requirements.txt requirements.txt
RUN uv pip install --upgrade -r requirements.txt

WORKDIR ${SOURCE_FOLDER}/ibek-support

COPY ibek-support/_ansible _ansible
ENV PATH=$PATH:${SOURCE_FOLDER}/ibek-support/_ansible

# emqComposite: the EMQ triplet's polarity sequence, an SNL program (README
# "emqComposite"). No upstream module exists; the source is in this repository
# (emqComposite/), copied into place before ansible.sh so the recipe builds it
# instead of cloning (as ioc-motion's motorVelmex).
COPY ibek-support-geecs/emqComposite/ ${SOURCE_FOLDER}/ibek-support-geecs/emqComposite/
COPY emqComposite/ /epics/support/emqComposite/
RUN ansible.sh emqComposite

# get the ioc source and build it
COPY ioc ${SOURCE_FOLDER}/ioc
RUN ansible.sh ioc

# generate a manifest of installed EPICS modules and python packages
# IOC_VERSION is declared here, not earlier: every RUN after an ARG sees it,
# so a new value (each branch or tag) would rebuild all the steps above
ARG IOC_VERSION=unknown
COPY scripts/generate_manifest.py /tmp/generate_manifest.py
RUN python3 /tmp/generate_manifest.py "${IOC_VERSION}"

##### runtime preparation stage ################################################
FROM developer AS runtime_prep

# get the products from the build stage and reduce to runtime assets only
# /python is created by uv and is needed in the runtime target
# /epics/versions.json is the manifest of support module and python versions
RUN ibek ioc extract-runtime-assets /assets /python /epics/versions.json

##### runtime stage ############################################################
FROM ${RUNTIME} AS runtime

# get runtime assets from the preparation stage
COPY --from=runtime_prep /assets /

# install runtime system dependencies, collected from install.sh scripts
RUN ibek support apt-install-runtime-packages

# emq_composite (uid/gid 1990) is the user the EMQ composite IOC runs as
# (geecs-epics emqtest1's compose `user:`), so that its CA links write under
# a name of their own. plctest1 grants coils 15-17 to that name only, and
# emqtest1-ps its setpoints. root, which every other container runs as, gets
# neither. start.sh's `ibek runtime generate2` removes and recreates
# /epics/runtime, so the user owns /epics itself (not recursively) and the
# runtime folder. The image's default user stays root for every other
# instance.
RUN groupadd --system --gid 1990 emq_composite \
    && useradd --system --uid 1990 --gid 1990 --no-create-home --shell /usr/sbin/nologin emq_composite \
    && mkdir -p /epics/runtime \
    && chown emq_composite:emq_composite /epics /epics/runtime

# launch the startup script with stdio-expose to allow console connections
CMD ["bash", "-c", "${IOC}/start.sh"]
