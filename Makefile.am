ACLOCAL_AMFLAGS = -I m4

EXTRA_DIST= \
 autogen.sh \
 emane_node_director \
 scripts \
 Copyright \
 debian \
 $(PACKAGE).spec.in \
 setup.py.in

all-local: setup.py
	$(PYTHON) setup.py build


clean-local: setup.py
	$(PYTHON) setup.py clean
	-rm -rf python*.egg-info
	-rm -rf build
	-rm -rf dist
	-rm -f emane-node-director*tar.gz
	-rm -f setup.py
	-rm -f MANIFEST
	-rm -f installed.txt
	-rm -f `find . -name "*\.pyc"`
	-rm -f `find . -name "*~"`
	-rm -rf .debbuild
	-rm -rf .rpmbuild
	-rm -rf $(PACKAGE).spec

install: setup.py
	$(PYTHON) setup.py install \
	    -O1 \
            --record .installedfiles \
            --prefix=$(prefix) \
            --exec-prefix=$(exec_prefix) \
            $(if $(DESTDIR),--root=$(DESTDIR)) \
            $(if $(subst false,,$(HAVE_DEB)),--install-layout=deb)

uninstall:
	if test -f .installedfiles; then xargs -a .installedfiles rm -f; fi

spec: setup.py
	$(PYTHON) setup.py bdist_rpm --spec-only


edit = sed \
        -e 's|@VERSION[@]|$(VERSION)|g' \
        -e 's|@DEBIAN_VERSION[@]|$(DEBIAN_VERSION)|g' \
        -e 's|@DATE_RFC2822[@]|$(DATE_RFC2822)|g'

if HAVE_RPMBUILD

RPMARCH=`arch`

rpm: $(PACKAGE).spec dist-gzip
	mkdir -p .rpmbuild/BUILD \
	.rpmbuild/SPECS \
	.rpmbuild/SOURCES \
	.rpmbuild/SRPMS \
	.rpmbuild/RPMS/noarch \
	.rpmbuild/tmp
	cp -f $(PACKAGE)-$(VERSION).tar.gz .rpmbuild/SOURCES
	rpmbuild --clean -ba $(top_srcdir)/$(PACKAGE).spec --target $(RPMARCH) \
    --define "_topdir $$PWD/.rpmbuild" \
    --define "_tmppath $$PWD/.rpmbuild/tmp"
	@echo "============================================================"
	@echo "RPMs located in .rpmbuild/RPMS and .rpmbuild/SRPMS"

endif

if HAVE_DEB

deb: dist-gzip
	rm -rf .debbuild
	mkdir -p .debbuild
	cp -f $(PACKAGE)-$(VERSION).tar.gz .debbuild
	cd .debbuild && tar xzf $(PACKAGE)-$(VERSION).tar.gz
	$(edit) .debbuild/$(PACKAGE)-$(VERSION)/debian/changelog.in > \
          .debbuild/$(PACKAGE)-$(VERSION)/debian/changelog
	cd .debbuild/$(PACKAGE)-$(VERSION) && dpkg-buildpackage -F -us -uc
	@echo "============================================================"
	@echo "debs located in .debbuild"
cleandeb:
	dh_clean
else

cleandeb:

endif

$(PACKAGE).spec: $(PACKAGE).spec.in
	if test -f $@; then chmod u+w $@; fi
	$(edit) $< > $@
	chmod g-w,u-w $@

setup.py: setup.py.in
	if test -f $@; then chmod u+w $@; fi
	$(edit) $< > $@
	chmod g-w,u-w $@
