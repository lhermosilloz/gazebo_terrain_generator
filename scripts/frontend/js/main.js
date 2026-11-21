// Main application entry point
(function () {
    'use strict';

    let map = null;
    let mapboxApiKey = null;
    let centerMarker = null;
    let draw = null;
    let coordinateOverlays = [];
    let showCoordinates = false;
    let vertexDeletePopup = null;
    let selectedVertex = null; // Store {featureId, coordIndex}
    let mouseDownOnVertex = false;
    let gridVisible = false;
    let generationCancelled = false;
    let pollingTimer = null;
    const GRID_LAYER_ID = 'grid-preview';
    const STORAGE_KEY = 'gazebo_terrain_generator_settings';

    const DEFAULT_CONFIG = {
        zoomLevel: 17,
        includeBuildings: true,
        includeHelipad: false,
        helipadHeight: 3,
        tileSource: 'https://mt0.google.com/vt?lyrs=s&x={x}&s=&y={y}&z={z}',
        parallelDownloads: 4,
        gazeboVersion: 'harmonic',
        targetHeightmapSize: 'auto',
        // Multi-spawn: when helipads are on, one per spawn point (false = spawn 1 only)
        spawnHelipadAll: true,
        // Ground friction: 'default' keeps Gazebo's default friction (mu = 1)
        frictionPreset: 'default',
        frictionCoefficient: 1,
        // Smooth roads: flatten road corridors in the heightmap
        roadsSmooth: true,
        roadsSmoothingLength: 80,
        roadsShoulderWidth: 4
    };

    const config = loadConfig();

    const TILE_SOURCES = [
        { label: 'Mapbox Satellite', url: 'https://api.mapbox.com/v4/mapbox.satellite/{z}/{x}/{y}@2x.jpg?access_token={key}' },
        null,
        { label: 'Bing Maps', url: 'http://ecn.t0.tiles.virtualearth.net/tiles/r{quad}.jpeg?g=129&mkt=en&stl=H' },
        { label: 'Bing Maps Satellite', url: 'http://ecn.t0.tiles.virtualearth.net/tiles/a{quad}.jpeg?g=129&mkt=en&stl=H' },
        { label: 'Bing Maps Hybrid', url: 'http://ecn.t0.tiles.virtualearth.net/tiles/h{quad}.jpeg?g=129&mkt=en&stl=H' },
        null,
        { label: 'Google Maps', url: 'https://mt0.google.com/vt?lyrs=m&x={x}&s=&y={y}&z={z}' },
        { label: 'Google Maps Satellite', url: 'https://mt0.google.com/vt?lyrs=s&x={x}&s=&y={y}&z={z}' },
        { label: 'Google Maps Hybrid', url: 'https://mt0.google.com/vt?lyrs=h&x={x}&s=&y={y}&z={z}' },
        { label: 'Google Maps Terrain', url: 'https://mt0.google.com/vt?lyrs=p&x={x}&s=&y={y}&z={z}' },
        null,
        { label: 'Open Street Maps', url: 'https://a.tile.openstreetmap.org/{z}/{x}/{y}.png' },
        { label: 'Open Cycle Maps', url: 'http://a.tile.opencyclemap.org/cycle/{z}/{x}/{y}.png' },
        null,
        { label: 'ESRI World Imagery', url: 'http://services.arcgisonline.com/arcgis/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}' },
        { label: 'Wikimedia Maps', url: 'https://maps.wikimedia.org/osm-intl/{z}/{x}/{y}.png' },
        null,
        { label: 'Carto Light', url: 'http://cartodb-basemaps-c.global.ssl.fastly.net/light_all/{z}/{x}/{y}.png' },
        { label: 'Stamen Toner B&W', url: 'http://a.tile.stamen.com/toner/{z}/{x}/{y}.png' },
    ];

    function loadConfig() {
        try {
            const stored = localStorage.getItem(STORAGE_KEY);
            if (stored) {
                return Object.assign({}, DEFAULT_CONFIG, JSON.parse(stored));
            }
        } catch (e) {
            console.warn('Failed to load settings from localStorage:', e);
        }
        return Object.assign({}, DEFAULT_CONFIG);
    }

    function saveConfig() {
        try {
            localStorage.setItem(STORAGE_KEY, JSON.stringify(config));
        } catch (e) {
            console.warn('Failed to save settings to localStorage:', e);
        }
    }


    function applyConfigToForm() {
        document.getElementById('setting-zoom-level').value = config.zoomLevel;
        document.getElementById('setting-include-buildings').checked = config.includeBuildings;
        document.getElementById('setting-include-helipad').checked = config.includeHelipad;
        document.getElementById('setting-helipad-height').value = config.helipadHeight;
        document.getElementById('setting-helipad-height-field').style.display = config.includeHelipad ? '' : 'none';
        document.getElementById('setting-tile-source').value = config.tileSource;
        document.getElementById('setting-parallel-downloads').value = config.parallelDownloads;
        document.getElementById('setting-gazebo-version').value = config.gazeboVersion;
        document.getElementById('setting-target-heightmap-size').value = config.targetHeightmapSize;
        applyFrictionConfigToForm();
        applyRoadsConfigToForm();
        updateMapboxKeyStatus(loadMapboxKey());

        const matchedSource = TILE_SOURCES.find(s => s && s.url === config.tileSource);
        document.getElementById('setting-source-display').textContent =
            matchedSource ? matchedSource.label : 'Custom';
        applySpawnConfigToForm();
    }

    const MAPBOX_KEY_STORAGE_KEY = 'gazebo_terrain_generator_mapbox_key';

    function loadMapboxKey() {
        return localStorage.getItem(MAPBOX_KEY_STORAGE_KEY) || '';
    }

    function saveMapboxKey(key) {
        localStorage.setItem(MAPBOX_KEY_STORAGE_KEY, key);
    }

    function openApikeyModal(onSave) {
        const overlay = document.getElementById('apikey-modal-overlay');
        const input = document.getElementById('apikey-modal-input');
        const errorEl = document.getElementById('apikey-modal-error');
        const confirmBtn = document.getElementById('apikey-modal-confirm');
        const cancelBtn = document.getElementById('apikey-modal-cancel');

        const hasExistingKey = !!loadMapboxKey();
        input.value = hasExistingKey ? loadMapboxKey() : '';
        errorEl.style.display = 'none';
        cancelBtn.style.display = hasExistingKey ? '' : 'none';

        overlay.classList.add('open');
        input.focus();

        async function save() {
            const key = input.value.trim();
            if (!key) return;

            // Mapbox GL refuses secret tokens, even though they pass the validation request below
            if (key.startsWith('sk.')) {
                errorEl.textContent = 'This is a secret token (sk.…). The map needs a public token, which starts with pk.';
                errorEl.style.display = 'block';
                return;
            }

            confirmBtn.disabled = true;
            confirmBtn.innerHTML = '<i class="fas fa-spinner fa-spin"></i> Validating...';
            errorEl.style.display = 'none';

            try {
                const resp = await fetch(
                    `https://api.mapbox.com/styles/v1/mapbox/satellite-v9?access_token=${encodeURIComponent(key)}`
                );
                if (resp.status === 401) {
                    throw new Error('Invalid API key — please check and try again.');
                } else if (!resp.ok) {
                    throw new Error(`Validation failed (HTTP ${resp.status}).`);
                }
            } catch (e) {
                const msg = (e instanceof TypeError)
                    ? 'Could not reach Mapbox API - check if your key is valid and make sure you are connected to internet.'
                    : e.message;
                errorEl.textContent = msg;
                errorEl.style.display = 'block';
                return;
            } finally {
                confirmBtn.disabled = false;
                confirmBtn.innerHTML = '<i class="fas fa-check"></i> Save';
            }

            // Outside the try: an error from onSave must not be written into the modal we just closed
            overlay.classList.remove('open');
            saveMapboxKey(key);
            updateMapboxKeyStatus(key);
            if (onSave) onSave(key);
        }

        function cancel() {
            overlay.classList.remove('open');
        }

        confirmBtn.onclick = save;
        cancelBtn.onclick = cancel;
    }

    function updateMapboxKeyStatus(key) {
        const btn = document.getElementById('setting-mapbox-key-btn');
        const status = document.getElementById('setting-mapbox-key-status');
        if (key) {
            status.textContent = 'Configured';
            btn.classList.add('key-set');
        } else {
            status.textContent = 'Not set';
            btn.classList.remove('key-set');
        }
    }

    // Initialize the application
    async function init() {
        mapboxApiKey = loadMapboxKey();
        // A secret token saved earlier can't be used by Mapbox GL; ask for a public one
        if (mapboxApiKey.startsWith('sk.')) mapboxApiKey = '';
        updateMapboxKeyStatus(mapboxApiKey);

        if (mapboxApiKey) {
            try {
                const resp = await fetch(
                    `https://api.mapbox.com/styles/v1/mapbox/satellite-v9?access_token=${encodeURIComponent(mapboxApiKey)}`
                );
                if (!resp.ok) mapboxApiKey = '';
            } catch (e) {
                mapboxApiKey = '';
            }
        }

        if (!mapboxApiKey) {
            openApikeyModal(function (key) {
                mapboxApiKey = key;
                mapboxgl.accessToken = key;
                initializeMap();
            });
            return;
        }
        mapboxgl.accessToken = mapboxApiKey;
        initializeMap();
    }

    // Initialize Mapbox map
    function initializeMap() {
        try {
            map = new mapboxgl.Map({
                container: 'map',
                style: 'mapbox://styles/mapbox/satellite-streets-v12',
                center: [-122.4194, 37.7749], // Default: San Francisco
                zoom: 12
            });
        } catch (e) {
            // The constructor throws synchronously, e.g. when WebGL is unavailable
            map = null;
            showError('Could not create the map: ' + e.message);
            return;
        }

        // Add navigation controls
        map.addControl(new mapboxgl.NavigationControl(), 'top-right');

        // Add scale control
        map.addControl(new mapboxgl.ScaleControl(), 'bottom-left');

        map.on('load', function () {
            console.log('Map loaded successfully');

            // Initialize drawing tools
            initializeDrawing();

            // Add center marker after map loads
            addCenterMarker();
            document.getElementById('coordinate-input').value = 'San Francisco';

            // Setup search button
            setupSearchButton();

            // Setup draw controls
            setupDrawControls();

            // Setup settings panel
            setupSettingsPanel();

            // Setup additional spawn points (multi-spawn)
            setupSpawnPoints();

            // Setup generate button
            document.getElementById('generate-btn').addEventListener('click', function () {
                if (validateForGeneration()) {
                    generateTerrain();
                }
            });

        });

        // Failed tile requests repeat on every pan/zoom; toast each kind once and log the rest
        const shownMapErrors = new Set();
        map.on('error', function (e) {
            console.error('Map error:', e);
            const err = e.error || {};
            let errorMsg = err.message || e.message || '';
            let key = errorMsg;
            if (err.status) {
                // HTTP failure: Mapbox GL uses the response statusText as the message,
                // which is empty over HTTP/2, so report the status and URL (without the token)
                const url = (err.url || '').split('?')[0];
                errorMsg = `HTTP ${err.status} loading ${url}${errorMsg ? ' (' + errorMsg + ')' : ''}`;
                key = `${err.status} ${e.sourceId || url.replace(/\/\d+\/\d+\/\d+.*$/, '')}`;
            }
            if (shownMapErrors.has(key)) return;
            shownMapErrors.add(key);
            showError('Map error: ' + (errorMsg || 'Unknown error'));
        });
    }

    // Add draggable marker at map center
    function addCenterMarker() {
        const center = map.getCenter();

        // Create draggable marker
        centerMarker = new mapboxgl.Marker({
            draggable: true,
            color: '#e74c3c'
        })
            .setLngLat([center.lng, center.lat])
            .addTo(map);

        // Update coordinate input with initial position
        updateCoordinateInput(center.lat, center.lng);

        // Listen for drag events
        centerMarker.on('drag', function () {
            const lngLat = centerMarker.getLngLat();
            updateCoordinateInput(lngLat.lat, lngLat.lng);
        });

        centerMarker.on('dragend', function () {
            const lngLat = centerMarker.getLngLat();
            console.log('Marker moved to:', lngLat.lat, lngLat.lng);
        });
    }

    // Update coordinate input field
    function updateCoordinateInput(lat, lng) {
        const input = document.getElementById('coordinate-input');
        if (input) {
            input.value = `${lat.toFixed(6)}, ${lng.toFixed(6)}`;
        }
    }

    // Setup search button functionality
    function setupSearchButton() {
        const searchButton = document.getElementById('search-button');
        const input = document.getElementById('coordinate-input');

        if (searchButton && input) {
            searchButton.addEventListener('click', function () {
                searchLocation();
            });

            input.addEventListener('keypress', function (e) {
                if (e.key === 'Enter') {
                    searchLocation();
                }
            });
        }
    }

    // Search for coordinates or place name
    async function searchLocation() {
        const input = document.getElementById('coordinate-input');
        const value = input.value.trim();
        if (!value) return;

        // Try coordinate parse first (format: lat, lng)
        const parts = value.split(',').map(s => parseFloat(s.trim()));
        if (parts.length === 2 && !isNaN(parts[0]) && !isNaN(parts[1])) {
            flyToLocation(parts[0], parts[1]);
            return;
        }

        // Fall back to Mapbox Geocoding API
        try {
            const url = `https://api.mapbox.com/geocoding/v5/mapbox.places/${encodeURIComponent(value)}.json?limit=1&access_token=${mapboxApiKey}`;
            const resp = await fetch(url);
            const data = await resp.json();
            if (data.features && data.features.length > 0) {
                const [lng, lat] = data.features[0].center;
                flyToLocation(lat, lng);
            } else {
                showError('Location not found. Try a different name or use lat, lng format.');
            }
        } catch (e) {
            showError('Geocoding failed: ' + e.message);
        }
    }

    function flyToLocation(lat, lng) {
        draw.deleteAll();
        clearCoordinateOverlays();
        updateTextureSizeEstimate();
        setGridVisible(false);
        map.flyTo({ center: [lng, lat], zoom: 15 });
        if (centerMarker) {
            centerMarker.setLngLat([lng, lat]);
        }
        // A new location starts a new selection: drop additional spawn points too
        clearSpawnPoints();
    }

    // Initialize drawing tools
    function initializeDrawing() {
        draw = new MapboxDraw({
            displayControlsDefault: false,
            controls: {},
            defaultMode: 'simple_select'
        });

        map.addControl(draw, 'top-right');

        // Listen for draw events
        map.on('draw.create', onDrawCreate);
        map.on('draw.update', onDrawUpdate);
        map.on('draw.delete', onDrawDelete);
        map.on('draw.selectionchange', onSelectionChange);

        // Set up drag detection
        setupDragDetection();

        // Add keyboard listener for delete key
        document.addEventListener('keydown', handleKeyPress);

        // Add right-click handler for vertices
        setupVertexContextMenu();
    }

    function moveMarkerToCentroid(polygon) {
        const centroid = turf.centroid(polygon);
        const [lng, lat] = centroid.geometry.coordinates;
        centerMarker.setLngLat([lng, lat]);
        updateCoordinateInput(lat, lng);
    }

    // Handle draw create event
    function onDrawCreate(e) {
        setGridVisible(false);
        if (e.features.length > 0) {
            moveMarkerToCentroid(e.features[0]);
        }
        updateCoordinateOverlays();
        updateTextureSizeEstimate();
    }

    // Handle draw update event
    function onDrawUpdate(e) {
        if (e.features.length > 0) {
            const polygon = e.features[0];
            const markerPos = centerMarker.getLngLat();
            const markerPt = turf.point([markerPos.lng, markerPos.lat]);
            if (!turf.booleanPointInPolygon(markerPt, polygon)) {
                moveMarkerToCentroid(polygon);
            }
        }
        updateCoordinateOverlays();
        updateTextureSizeEstimate();
    }

    // Handle draw delete event
    function onDrawDelete(e) {
        console.log('Polygon deleted:', e.features);
        clearCoordinateOverlays();
        hideVertexDeletePopup();
        updateTextureSizeEstimate();
    }

    // Handle selection change
    function onSelectionChange(e) {
        // Remove old popup first
        if (vertexDeletePopup) {
            vertexDeletePopup.remove();
            vertexDeletePopup = null;
        }

        if (e.features.length > 0 && e.points && e.points.length > 0) {
            // A vertex is selected
            const point = e.points[0];
            const feature = e.features[0];

            console.log('Selection event:', e);
            console.log('Point:', point);
            console.log('Feature:', feature);

            // Find which vertex was clicked by matching coordinates
            const clickedCoord = point.geometry.coordinates;
            const featureCoords = feature.geometry.coordinates[0];

            let coordIndex = -1;
            for (let i = 0; i < featureCoords.length - 1; i++) { // -1 to skip closing point
                if (Math.abs(featureCoords[i][0] - clickedCoord[0]) < 0.000001 &&
                    Math.abs(featureCoords[i][1] - clickedCoord[1]) < 0.000001) {
                    coordIndex = i;
                    break;
                }
            }

            if (coordIndex >= 0) {
                selectedVertex = {
                    featureId: feature.id,
                    coordIndex: coordIndex
                };
                console.log('Vertex selected at index:', coordIndex);
                // Mark that mouse is down on a vertex
                mouseDownOnVertex = true;
                showVertexDeletePopup(point);
            }
        } else {
            // No vertex selected, clear everything
            selectedVertex = null;
            mouseDownOnVertex = false;
        }
    }

    // Setup drag detection
    function setupDragDetection() {
        map.on('mousemove', function (e) {
            if (mouseDownOnVertex && vertexDeletePopup) {
                console.log('Drag detected, hiding popup');
                hideVertexDeletePopup();
            }
        });

        map.on('mousedown', function (e) {
            if (gridVisible) {
                hideGrid();
            }
        })

        map.on('mouseup', function (e) {
            if (selectedVertex) {
                const feature = draw.get(selectedVertex.featureId);
                const coords = feature.geometry.coordinates[0][selectedVertex.coordIndex];
                showVertexDeletePopup({geometry: {coordinates: coords}});
            }
            if (gridVisible) {
                showGrid();
            }
            mouseDownOnVertex = false;
        });

        map.on('click', function (e) {
            if (draw.getAll().features.length === 0) {
                centerMarker.setLngLat(e.lngLat);
                updateCoordinateInput(e.lngLat.lat, e.lngLat.lng);
            }
        });
    }

    // Setup vertex context menu (right-click)
    function setupVertexContextMenu() {
        map.on('contextmenu', function (e) {
            // Check if we're clicking on a draw feature
            const features = map.queryRenderedFeatures(e.point);
            const isDrawFeature = features.some(f =>
                f.source === 'mapbox-gl-draw-cold' ||
                f.source === 'mapbox-gl-draw-hot'
            );

            if (isDrawFeature && selectedVertex) {
                e.preventDefault();
                deleteSelectedVertex();
            }
        });
    }

    // Handle keyboard events
    function handleKeyPress(e) {
        if (e.key === 'Delete' || e.key === 'Backspace') {
            if (selectedVertex) {
                e.preventDefault();
                deleteSelectedVertex();
            }
        }
    }

    // Show vertex delete popup
    function showVertexDeletePopup(point) {
        // Remove old popup if exists
        if (vertexDeletePopup) {
            vertexDeletePopup.remove();
            vertexDeletePopup = null;
        }

        const coords = [point.geometry.coordinates[0], point.geometry.coordinates[1]];

        vertexDeletePopup = new mapboxgl.Popup({
            closeButton: false,
            closeOnClick: false,
            className: 'vertex-delete-popup',
            anchor: 'top',
            offset: 15
        })
            .setLngLat(coords)
            .setHTML('<button id="delete-vertex-btn" class="delete-vertex-btn"><i class="fas fa-trash-alt"></i></button>')
            .addTo(map);

        // Add click handler to delete button
        setTimeout(function () {
            const deleteBtn = document.getElementById('delete-vertex-btn');
            if (deleteBtn) {
                deleteBtn.onclick = function (e) {
                    e.stopPropagation();
                    console.log('Delete button clicked, selectedVertex:', selectedVertex);
                    deleteSelectedVertex();
                };
            }
        }, 0);
    }

    // Hide vertex delete popup
    function hideVertexDeletePopup() {
        if (vertexDeletePopup) {
            vertexDeletePopup.remove();
            vertexDeletePopup = null;
        }
        //selectedVertex = null;
    }

    // Delete selected vertex
    function deleteSelectedVertex() {
        if (!selectedVertex) {
            console.log('No vertex selected');
            return;
        }

        console.log('Attempting to delete vertex:', selectedVertex);

        // Get the feature by ID
        const allFeatures = draw.getAll().features;
        const feature = allFeatures.find(f => f.id === selectedVertex.featureId);

        if (!feature) {
            console.log('Feature not found');
            return;
        }

        if (feature.geometry.type !== 'Polygon') {
            console.log('Selected feature is not a polygon');
            return;
        }

        const coordinates = feature.geometry.coordinates[0].slice(); // Make a copy

        // Don't allow deletion if polygon would have less than 4 points (3 unique + closing)
        if (coordinates.length <= 4) {
            showError('Cannot delete vertex: polygon must have at least 3 vertices');
            return;
        }

        const coordIndex = selectedVertex.coordIndex;

        console.log('Deleting vertex at index:', coordIndex, 'from', coordinates.length, 'coordinates');

        // Remove the vertex
        coordinates.splice(coordIndex, 1);

        // Ensure the polygon is closed (first and last points are the same)
        if (coordIndex === 0) {
            coordinates[coordinates.length - 1] = [coordinates[0][0], coordinates[0][1]];
        }

        // Update the feature
        const featureId = feature.id;
        const updatedFeature = {
            id: featureId,
            type: 'Feature',
            geometry: {
                type: 'Polygon',
                coordinates: [coordinates]
            },
            properties: feature.properties
        };

        draw.delete(featureId);
        draw.add(updatedFeature);
        draw.changeMode('simple_select', {featureIds: [featureId]});

        hideVertexDeletePopup();
        updateCoordinateOverlays();

        console.log('Vertex deleted successfully. New coordinate count:', coordinates.length);
    }

    // Setup draw control buttons
    function setupDrawControls() {
        // Create custom control container
        const controlContainer = document.createElement('div');
        controlContainer.className = 'mapboxgl-ctrl mapboxgl-ctrl-group custom-draw-controls';

        // Draw polygon button
        const drawButton = document.createElement('button');
        drawButton.className = 'mapbox-gl-draw_ctrl-draw-btn';
        drawButton.title = 'Draw polygon';
        drawButton.innerHTML = '<i class="fas fa-draw-polygon"></i>';
        drawButton.onclick = function () {
            draw.deleteAll();
            clearCoordinateOverlays();
            updateTextureSizeEstimate();
            draw.changeMode('draw_polygon');
        };

        // Toggle coordinates button
        const coordButton = document.createElement('button');
        coordButton.className = 'mapbox-gl-draw_ctrl-draw-btn';
        coordButton.title = 'Toggle coordinate labels';
        coordButton.innerHTML = '<i class="fas fa-tag"></i>';
        coordButton.onclick = function () {
            showCoordinates = !showCoordinates;
            coordButton.style.backgroundColor = showCoordinates ? '#3bb2d0' : '';
            updateCoordinateOverlays();
        };

        // Grid preview button
        const gridButton = document.createElement('button');
        gridButton.className = 'mapbox-gl-draw_ctrl-draw-btn';
        gridButton.title = 'Toggle grid preview';
        gridButton.innerHTML = '<i class="fas fa-th"></i>';
        gridButton.onclick = function () {
            setGridVisible(!gridVisible, gridButton);
        };

        // Expose button so other functions can reset its style
        window._gridButton = gridButton;

        // Settings button
        const settingsButton = document.createElement('button');
        settingsButton.className = 'mapbox-gl-draw_ctrl-draw-btn';
        settingsButton.title = 'Settings';
        settingsButton.innerHTML = '<i class="fas fa-cog"></i>';
        settingsButton.onclick = function () {
            const panel = document.getElementById('settings-panel');
            panel.classList.toggle('open');
            document.getElementById('map').classList.toggle('settings-open', panel.classList.contains('open'));
        };

        controlContainer.appendChild(drawButton);
        controlContainer.appendChild(coordButton);
        controlContainer.appendChild(gridButton);
        controlContainer.appendChild(settingsButton);

        // Add to map
        map.getContainer().querySelector('.mapboxgl-ctrl-top-right').appendChild(controlContainer);
    }

    // Update coordinate overlays
    function updateCoordinateOverlays() {
        clearCoordinateOverlays();

        if (!showCoordinates) return;

        const features = draw.getAll().features;
        features.forEach(function (feature) {
            if (feature.geometry.type === 'Polygon') {
                const coordinates = feature.geometry.coordinates[0];
                coordinates.forEach(function (coord, index) {
                    // Skip last coordinate (same as first)
                    if (index === coordinates.length - 1) return;

                    const popup = new mapboxgl.Popup({
                        closeButton: false,
                        closeOnClick: false,
                        className: 'coordinate-label'
                    })
                        .setLngLat(coord)
                        .setHTML(`${coord[1].toFixed(6)}, ${coord[0].toFixed(6)}`)
                        .addTo(map);

                    coordinateOverlays.push(popup);
                });
            }
        });
    }

    // Clear coordinate overlays
    function clearCoordinateOverlays() {
        coordinateOverlays.forEach(function (popup) {
            popup.remove();
        });
        coordinateOverlays = [];
    }

    // Validate inputs before generation
    function validateForGeneration() {
        const features = draw.getAll().features;

        if (features.length === 0) {
            showError('Draw a polygon on the map first.');
            return false;
        }

        const polygon = features[0];

        // Check convexity by comparing polygon area to its convex hull area
        const hull = turf.convex(polygon);
        if (!hull) {
            showError('Invalid polygon shape.');
            return false;
        }
        const areaDiff = Math.abs(turf.area(hull) - turf.area(polygon)) / turf.area(polygon);
        if (areaDiff > 0.001) {
            showError('Polygon must be convex. Remove inward vertices to fix it.');
            return false;
        }

        // Check pivot point is inside the polygon
        const lngLat = centerMarker.getLngLat();
        const pivotPoint = turf.point([lngLat.lng, lngLat.lat]);
        if (!turf.booleanPointInPolygon(pivotPoint, polygon)) {
            showError('The pivot point (red marker) must be inside the polygon.');
            return false;
        }

        // Every additional spawn point must be inside the polygon too (multi-spawn)
        if (!validateSpawnPoints(polygon)) {
            return false;
        }

        return true;
    }

    // --- Generation flow ---

    async function generateTerrain() {
        // Reverse geocode center marker for suggested name
        const lngLat = centerMarker.getLngLat();
        let suggested = 'world';
        try {
            suggested = await fetchModelName(lngLat.lng, lngLat.lat);
        } catch (e) {
            console.warn('Reverse geocoding failed, using fallback name');
        }

        const modelName = await showNameModal(suggested);
        if (!modelName) return; // User cancelled

        await startGeneration(modelName);
    }

    async function fetchModelName(lng, lat) {
        const url = `https://api.mapbox.com/geocoding/v5/mapbox.places/${lng},${lat}.json?types=locality,place,region&limit=1&access_token=${mapboxApiKey}`;
        const resp = await fetch(url);
        const data = await resp.json();
        if (data.features && data.features.length > 0) {
            const text = data.features[0].text || data.features[0].place_name.split(',')[0];
            const slug = text.toLowerCase().replace(/[^a-z0-9]+/g, '_').replace(/^_+|_+$/g, '');
            return slug.charAt(0).toUpperCase() + slug.slice(1);
        }
        return 'World';
    }

    function showNameModal(suggested) {
        return new Promise(function (resolve) {
            const overlay = document.getElementById('name-modal-overlay');
            const input = document.getElementById('name-modal-input');
            const confirmBtn = document.getElementById('name-modal-confirm');
            const cancelBtn = document.getElementById('name-modal-cancel');

            input.value = suggested;
            overlay.classList.add('open');
            input.focus();
            input.select();

            function confirm() {
                const name = input.value.trim();
                if (!name) return;
                cleanup();
                resolve(name);
            }

            function cancel() {
                cleanup();
                resolve(null);
            }

            function onKeyDown(e) {
                if (e.key === 'Enter') confirm();
                if (e.key === 'Escape') cancel();
            }

            function cleanup() {
                overlay.classList.remove('open');
                confirmBtn.removeEventListener('click', confirm);
                cancelBtn.removeEventListener('click', cancel);
                input.removeEventListener('keydown', onKeyDown);
            }

            confirmBtn.addEventListener('click', confirm);
            cancelBtn.addEventListener('click', cancel);
            input.addEventListener('keydown', onKeyDown);
        });
    }

    async function startGeneration(modelName) {
        generationCancelled = false;

        const timestamp = Date.now().toString();
        const outputFile = '{z}/{x}/{y}.png';
        const zoomLevel = config.zoomLevel;
        const source = config.tileSource;
        const includeBuildings = config.includeBuildings;
        const parallelDownloads = config.parallelDownloads;

        // Get polygon vertices — bounds/center computed server-side
        const polygon = draw.getAll().features[0];
        const coords = polygon.geometry.coordinates[0];
        const lngLat = centerMarker.getLngLat();
        const launchLocation = [lngLat.lng, lngLat.lat];
        // Multi-spawn: every spawn point, primary (= launchLocation) first
        const spawnPointsJson = JSON.stringify(getSpawnPointsPayload());

        const tiles = getTiles(zoomLevel);

        showConsole();
        logToConsole(`${tiles.length} tiles queued at zoom ${zoomLevel}`, 'info');
        updateConsoleProgress(0, tiles.length);

        // POST /start-download
        try {
            const startData = new FormData();
            startData.append('maxZoom', zoomLevel);
            startData.append('mapName', modelName);
            startData.append('outputFile', outputFile);
            startData.append('timestamp', timestamp);
            startData.append('polygonVertices', JSON.stringify(coords));
            startData.append('launchLocation', launchLocation.join(','));
            startData.append('includeBuildings', includeBuildings);
            startData.append('includeHelipad', config.includeHelipad);
            startData.append('helipadHeight', config.helipadHeight);
            startData.append('gazeboVersion', config.gazeboVersion);
            startData.append('frictionPreset', config.frictionPreset);
            startData.append('frictionCoefficient', config.frictionCoefficient);
            startData.append('source', source);
            startData.append('spawnPoints', spawnPointsJson);
            startData.append('spawnHelipadAll', config.spawnHelipadAll);
            appendRoadsSettings(startData);
            const startResp = await fetch('/start-download', { method: 'POST', body: startData });
            const startResult = await startResp.json();
            if (startResult.code !== 200) throw new Error('start-download failed: ' + startResult.message);
        } catch (e) {
            logToConsole('Error: ' + e.message, 'error');
            setConsoleStatus('failed');
            return;
        }

        // Download tiles with concurrency
        let completed = 0;
        await runWithConcurrency(tiles, parallelDownloads, async function (tile) {
            if (generationCancelled) return;

            const tileData = new FormData();
            tileData.append('x', tile.x);
            tileData.append('y', tile.y);
            tileData.append('z', tile.z);
            tileData.append('mapName', modelName);
            tileData.append('source', source);
            tileData.append('mapboxApiKey', mapboxApiKey);

            try {
                const resp = await fetch('/download-tile', { method: 'POST', body: tileData });
                const result = await resp.json();
                completed++;
                updateConsoleProgress(completed, tiles.length);
                logToConsole(`[${tile.x},${tile.y},${tile.z}] ${result.message}`);
            } catch (e) {
                logToConsole(`[${tile.x},${tile.y},${tile.z}] Error: ${e.message}`, 'error');
            }
        });

        if (generationCancelled) return;

        // POST /end-download
        logToConsole('All tiles downloaded. Building Gazebo world...', 'info');
        setConsoleStatus('Generating...');

        try {
            const endData = new FormData();
            endData.append('mapName', modelName);
            endData.append('maxZoom', zoomLevel);
            endData.append('polygonVertices', JSON.stringify(coords));
            endData.append('includeBuildings', includeBuildings);
            endData.append('includeHelipad', config.includeHelipad);
            endData.append('helipadHeight', config.helipadHeight);
            endData.append('gazeboVersion', config.gazeboVersion);
            endData.append('targetHeightmapSize', config.targetHeightmapSize);
            endData.append('frictionPreset', config.frictionPreset);
            endData.append('frictionCoefficient', config.frictionCoefficient);
            endData.append('mapboxApiKey', mapboxApiKey);
            endData.append('spawnPoints', spawnPointsJson);
            endData.append('spawnHelipadAll', config.spawnHelipadAll);
            appendRoadsSettings(endData);
            const endResp = await fetch('/end-download', { method: 'POST', body: endData });
            const endResult = await endResp.json();
            if (endResult.code !== 200) throw new Error('end-download failed');
        } catch (e) {
            logToConsole('Error: ' + e.message, 'error');
            setConsoleStatus('failed');
            return;
        }

        pollGenerationStatus(modelName);
    }

    async function pollGenerationStatus(mapName) {
        if (generationCancelled) return;
        try {
            const resp = await fetch('/task-status');
            const data = await resp.json();
            const status = data.message.status;
            const messages = data.message.messages || [];

            if (status === 'completed') {
                messages.forEach(msg => logToConsole(msg, 'success'));
                setConsoleStatus('completed');
                setConsoleCloseMode(mapName);
            } else if (status === 'failed') {
                messages.forEach(msg => logToConsole(msg, 'error'));
                setConsoleStatus('failed');
                setConsoleCloseMode();
            } else {
                messages.forEach(msg => logToConsole(msg));
                pollingTimer = setTimeout(() => pollGenerationStatus(mapName), 1000);
            }
        } catch (e) {
            logToConsole('Error polling status: ' + e.message, 'error');
            pollingTimer = setTimeout(() => pollGenerationStatus(mapName), 5000);
        }
    }

    // Returns tiles as {x, y, z} for a given zoom level
    function getTiles(zoomLevel) {
        const polygon = draw.getAll().features[0];
        const coords = polygon.geometry.coordinates[0];
        const lngs = coords.map(c => c[0]);
        const lats = coords.map(c => c[1]);
        const TY = lat2tile(Math.max(...lats), zoomLevel);
        const BY = lat2tile(Math.min(...lats), zoomLevel);
        const LX = long2tile(Math.min(...lngs), zoomLevel);
        const RX = long2tile(Math.max(...lngs), zoomLevel);

        const tiles = [];
        for (let y = TY; y <= BY; y++) {
            for (let x = LX; x <= RX; x++) {
                const tileRect = getTileRect(x, y, zoomLevel);
                const tilePolygon = turf.polygon([tileRect]);
                if (!turf.booleanDisjoint(tilePolygon, polygon)) {
                    tiles.push({ x, y, z: zoomLevel });
                }
            }
        }
        return tiles;
    }

    async function runWithConcurrency(items, limit, task) {
        let index = 0;
        async function worker() {
            while (index < items.length && !generationCancelled) {
                await task(items[index++]);
            }
        }
        const workers = Array.from({ length: Math.min(limit, items.length) }, worker);
        await Promise.all(workers);
    }

    // --- Console UI ---

    function showConsole() {
        document.getElementById('console-log').innerHTML = '';
        document.getElementById('console-progress-fill').style.width = '0%';
        document.getElementById('console-progress-label').textContent = '0 / 0 tiles';
        document.getElementById('console-status-badge').className = 'console-status-badge';
        document.getElementById('console-status-badge').textContent = 'Running';
        document.getElementById('console-download-btn').style.display = 'none';
        document.getElementById('console-intermediary-wrap').style.display = 'none';
        document.getElementById('console-intermediary-chk').checked = false;
        const stopBtn = document.getElementById('console-stop-btn');
        stopBtn.className = 'console-stop-btn';
        stopBtn.innerHTML = '<i class="fas fa-stop"></i> Stop';
        stopBtn.onclick = function () {
            generationCancelled = true;
            if (pollingTimer) clearTimeout(pollingTimer);
            logToConsole('Stopping...', 'error');
            setConsoleStatus('stopped');
            setConsoleCloseMode();
        };
        document.getElementById('console-overlay').classList.add('open');
    }

    function logToConsole(message, type) {
        const log = document.getElementById('console-log');
        const line = document.createElement('p');
        if (type) line.className = 'log-' + type;
        line.textContent = '> ' + message;
        log.appendChild(line);
        log.scrollTop = log.scrollHeight;
    }

    function updateConsoleProgress(completed, total) {
        const pct = total > 0 ? (completed / total) * 100 : 0;
        document.getElementById('console-progress-fill').style.width = pct + '%';
        document.getElementById('console-progress-label').textContent = `${completed} / ${total} tiles`;
    }

    function setConsoleStatus(status) {
        const badge = document.getElementById('console-status-badge');
        const labels = { completed: 'Completed', failed: 'Failed', stopped: 'Stopped', 'Generating...': 'Generating...' };
        badge.className = 'console-status-badge ' + status;
        badge.textContent = labels[status] || status;
    }

    function setConsoleCloseMode(mapName) {
        const btn = document.getElementById('console-stop-btn');
        btn.className = 'console-stop-btn close';
        btn.innerHTML = '<i class="fas fa-times"></i> Close';
        btn.onclick = function () {
            document.getElementById('console-overlay').classList.remove('open');
        };

        const dlBtn = document.getElementById('console-download-btn');
        const dlWrap = document.getElementById('console-intermediary-wrap');
        if (mapName) {
            dlWrap.style.display = '';
            dlBtn.style.display = '';
            dlBtn.onclick = function () {
                const includeIntermediary = document.getElementById('console-intermediary-chk').checked;
                window.location.href = '/download-world?mapName=' + encodeURIComponent(mapName)
                    + '&includeIntermediary=' + includeIntermediary;
            };
        } else {
            dlWrap.style.display = 'none';
            dlBtn.style.display = 'none';
        }
    }

    // Populate Target Heightmap Size dropdown from backend (single source of truth)
    async function populateHeightmapSizeDropdown() {
        try {
            const resp = await fetch('/valid-heightmap-sizes');
            const result = await resp.json();
            if (result.code === 200) {
                const select = document.getElementById('setting-target-heightmap-size');
                result.sizes.forEach(function (size) {
                    const option = document.createElement('option');
                    option.value = String(size);
                    option.textContent = size + ' x ' + size;
                    select.appendChild(option);
                });
                // Restore saved value now that options exist
                select.value = config.targetHeightmapSize;
            }
        } catch (e) {
            console.warn('Failed to load heightmap sizes:', e);
        }
    }

    // Last successful estimate result — re-rendered when dropdown selection changes
    let lastEstimateResult = null;

    // Render the estimate info div from a cached result object
    function renderEstimateInfo(result) {
        const infoEl = document.getElementById('texture-size-info');
        if (!result) return;
        const isAuto = config.targetHeightmapSize === 'auto';
        const resolvedSize = isAuto ? result.auto_heightmap_size : parseInt(config.targetHeightmapSize, 10);
        const modeLabel = isAuto ? 'Auto' : 'Fixed';
        const hmLine = 'Heightmap: ' + result.natural_heightmap_width + 'x' + result.natural_heightmap_height + 'px -> ' + modeLabel + ': ' + resolvedSize + 'x' + resolvedSize;
        const texLine = 'Texture: ~' + result.natural_texture_padded + 'x' + result.natural_texture_padded + 'px';

        let areaLine = '';
        if (result.areaSqM !== undefined) {
            areaLine = result.areaSqM >= 1e6
                ? 'Area: ' + (result.areaSqM / 1e6).toFixed(2) + ' km²'
                : 'Area: ' + (result.areaSqM / 1e4).toFixed(2) + ' ha';
        }
        const tilesLine = result.tileCount !== undefined
            ? 'Tiles: ' + result.tileCount + ' @ zoom ' + config.zoomLevel
            : '';

        infoEl.innerHTML = '<span class="texture-size-info-title">Estimated output sizes</span>'
            + [tilesLine, areaLine, hmLine, texLine].filter(Boolean).join('\n');
        infoEl.classList.add('visible');
    }

    // Fetch and display heightmap + texture size estimates from the server
    async function updateTextureSizeEstimate() {
        const infoEl = document.getElementById('texture-size-info');
        const features = draw.getAll().features;
        if (features.length === 0) {
            infoEl.innerHTML = '';
            infoEl.classList.remove('visible');
            lastEstimateResult = null;
            return;
        }
        const coords = features[0].geometry.coordinates[0];
        const data = new FormData();
        data.append('polygonVertices', JSON.stringify(coords));
        data.append('zoomLevel', config.zoomLevel);
        data.append('tileSource', config.tileSource);
        const infoData = new FormData();
        infoData.append('polygonVertices', JSON.stringify(coords));
        infoData.append('zoomLevel', config.zoomLevel);

        try {
            const [estimateResp, infoResp] = await Promise.all([
                fetch('/estimate-texture-sizes', { method: 'POST', body: data }),
                fetch('/polygon-info', { method: 'POST', body: infoData }),
            ]);
            const [result, info] = await Promise.all([estimateResp.json(), infoResp.json()]);
            if (result.code === 200) {
                if (info.code === 200) {
                    result.tileCount = info.tile_count;
                    result.areaSqM = info.area_sq_m;
                }
                lastEstimateResult = result;
                renderEstimateInfo(result);
            }
        } catch (e) {
            console.warn('Size estimate failed:', e);
        }
    }

    // Setup settings panel
    function setupSettingsPanel() {
        // Populate dropdowns from backend before applying saved config
        populateHeightmapSizeDropdown();

        // Apply loaded/default config to form fields
        applyConfigToForm();

        // Populate source presets dropdown
        const presetsDropdown = document.getElementById('source-presets-dropdown');
        TILE_SOURCES.forEach(function (source) {
            if (source === null) {
                const divider = document.createElement('div');
                divider.className = 'source-preset-divider';
                presetsDropdown.appendChild(divider);
            } else {
                const item = document.createElement('div');
                item.className = 'source-preset-item';
                item.textContent = source.label;
                item.addEventListener('click', function () {
                    document.getElementById('setting-tile-source').value = source.url;
                    document.getElementById('setting-source-display').textContent = source.label;
                    config.tileSource = source.url;
                    presetsDropdown.classList.remove('open');
                    saveConfig();
                });
                presetsDropdown.appendChild(item);
            }
        });

        // Toggle presets dropdown via display button or chevron
        function togglePresetsDropdown(e) {
            e.stopPropagation();
            presetsDropdown.classList.toggle('open');
        }
        document.getElementById('setting-source-display').addEventListener('click', togglePresetsDropdown);
        document.getElementById('setting-source-presets-btn').addEventListener('click', togglePresetsDropdown);

        // Close presets dropdown on outside click
        document.addEventListener('click', function (e) {
            if (!e.target.closest('#setting-source-display') &&
                !e.target.closest('#setting-source-presets-btn') &&
                !e.target.closest('#source-presets-dropdown')) {
                presetsDropdown.classList.remove('open');
            }
        });

        // Close panel button
        document.getElementById('settings-close-btn').addEventListener('click', function () {
            document.getElementById('settings-panel').classList.remove('open');
            document.getElementById('map').classList.remove('settings-open');
        });

        // Sync config on input change
        document.getElementById('setting-zoom-level').addEventListener('change', function () {
            config.zoomLevel = parseInt(this.value, 10) || 17;
            saveConfig();
            if (gridVisible) { showGrid(); }
            updateTextureSizeEstimate();
        });

        document.getElementById('setting-include-buildings').addEventListener('change', function () {
            config.includeBuildings = this.checked;
            saveConfig();
        });

        document.getElementById('setting-include-helipad').addEventListener('change', function () {
            config.includeHelipad = this.checked;
            document.getElementById('setting-helipad-height-field').style.display = this.checked ? '' : 'none';
            saveConfig();
        });

        document.getElementById('setting-helipad-height').addEventListener('change', function () {
            config.helipadHeight = parseFloat(this.value) || 3;
            saveConfig();
        });

        document.getElementById('setting-parallel-downloads').addEventListener('change', function () {
            config.parallelDownloads = parseInt(this.value, 10) || 4;
            saveConfig();
        });

        document.getElementById('setting-gazebo-version').addEventListener('change', function () {
            config.gazeboVersion = this.value;
            saveConfig();
        });

        document.getElementById('setting-target-heightmap-size').addEventListener('change', function () {
            config.targetHeightmapSize = this.value;
            saveConfig();
            renderEstimateInfo(lastEstimateResult);
        });

        setupFrictionSettings();

        setupRoadsSettings();

        document.getElementById('setting-mapbox-key-btn').addEventListener('click', function () {
            openApikeyModal(function (key) {
                mapboxApiKey = key;
                mapboxgl.accessToken = key;
                if (!map) initializeMap();
            });
        });

        document.getElementById('settings-revert-btn').addEventListener('click', function () {
            Object.assign(config, DEFAULT_CONFIG);
            localStorage.removeItem(STORAGE_KEY);
            applyConfigToForm();
            if (gridVisible) { showGrid(); }
            // Note: API key is intentionally not cleared by revert
        });
    }

    // ===== Multi-spawn: additional spawn points (feature: multi-spawn) =====
    // The red pin (centerMarker) is spawn 1, the world origin, exactly as before.
    // Additional spawn points are numbered markers (spawn 2..N) that must lie inside
    // the polygon. Like the polygon and the red pin, they are kept in memory for the
    // current selection only (not saved across reloads); a polygon redraw keeps them
    // (they are re-validated), a new location search clears them. The only saved
    // option is config.spawnHelipadAll.

    const SPAWN_MAX_POINTS = 32;     // primary included; matches MAX_SPAWN_POINTS on the server
    const SPAWN_HELIPAD_SIZE_M = 5;  // footprint of the Fuel helipad model, for overlap warnings
    let spawnExtras = [];            // [{ marker, yawDeg }] for spawn 2..N, in order
    let spawnPrimaryYawDeg = 0;      // yaw of spawn 1 (degrees, counter-clockwise from east)
    let spawnPlacing = false;        // true while waiting for a map click to place a point
    let spawnMapButton = null;
    let spawnPrimaryBadge = null;

    function setupSpawnPoints() {
        // Map control: "add spawn point" button next to the draw-polygon button
        const group = map.getContainer().querySelector('.custom-draw-controls');
        spawnMapButton = document.createElement('button');
        spawnMapButton.className = 'mapbox-gl-draw_ctrl-draw-btn';
        spawnMapButton.title = 'Add spawn point (then click inside the polygon)';
        spawnMapButton.innerHTML = '<i class="fas fa-map-pin"></i>';
        spawnMapButton.onclick = toggleSpawnPlacement;
        if (group) group.insertBefore(spawnMapButton, group.children[1] || null);

        document.getElementById('setting-spawn-add-btn').addEventListener('click', toggleSpawnPlacement);

        document.getElementById('setting-spawn-helipad-all').addEventListener('change', function () {
            config.spawnHelipadAll = this.checked;
            saveConfig();
        });
        // The "helipad at every spawn" option only matters when helipads are on
        document.getElementById('setting-include-helipad').addEventListener('change', updateSpawnHelipadFieldVisibility);

        map.on('click', onSpawnMapClick);
        document.addEventListener('keydown', function (e) {
            if (e.key === 'Escape' && spawnPlacing) stopSpawnPlacement();
        });

        // Re-validate and re-list after polygon edits (these run after the handlers
        // that may move the red pin to the polygon centroid) and red pin drags
        map.on('draw.create', refreshSpawnPoints);
        map.on('draw.update', refreshSpawnPoints);
        map.on('draw.delete', refreshSpawnPoints);
        // draw.create fires while Draw is still in draw_polygon mode, when
        // getSpawnPolygon() ignores the polygon; re-check once the mode has changed
        map.on('draw.modechange', refreshSpawnPoints);
        centerMarker.on('drag', updateSpawnListCoords);
        centerMarker.on('dragend', refreshSpawnPoints);
        releasePolygonOnPress(centerMarker.getElement());

        refreshSpawnPoints();
    }

    // MapboxDraw handles the same map mousedown as a marker drag, so pressing a
    // marker that sits on the selected polygon would drag the polygon along with
    // it. Deselect the polygon first (DOM listeners on the marker run before the
    // event reaches the map).
    function releasePolygonOnPress(el) {
        function release() {
            if (!draw) return;
            const mode = draw.getMode();
            if (mode === 'draw_polygon') return;
            if (mode !== 'simple_select' || draw.getSelectedIds().length > 0) {
                draw.changeMode('simple_select', { featureIds: [] });
            }
        }
        el.addEventListener('mousedown', release);
        el.addEventListener('touchstart', release, { passive: true });
    }

    function applySpawnConfigToForm() {
        document.getElementById('setting-spawn-helipad-all').checked = config.spawnHelipadAll;
        updateSpawnHelipadFieldVisibility();
    }

    function updateSpawnHelipadFieldVisibility() {
        const helipadOn = document.getElementById('setting-include-helipad').checked;
        document.getElementById('setting-spawn-helipad-all-field').style.display = helipadOn ? '' : 'none';
    }

    // The selection polygon, or null while there is none (or it is still being drawn)
    function getSpawnPolygon() {
        if (!draw || draw.getMode() === 'draw_polygon') return null;
        const features = draw.getAll().features;
        if (features.length === 0 || features[0].geometry.type !== 'Polygon') return null;
        if (features[0].geometry.coordinates[0].length < 4) return null;
        return features[0];
    }

    function isSpawnInside(lngLat, polygon) {
        return turf.booleanPointInPolygon(turf.point([lngLat.lng, lngLat.lat]), polygon);
    }

    function toggleSpawnPlacement() {
        if (spawnPlacing) {
            stopSpawnPlacement();
        } else {
            startSpawnPlacement();
        }
    }

    function startSpawnPlacement() {
        if (draw.getMode() === 'draw_polygon') {
            showError('Finish drawing the polygon first.');
            return;
        }
        if (!getSpawnPolygon()) {
            showError('Draw the polygon first, then add spawn points inside it.');
            return;
        }
        if (1 + spawnExtras.length >= SPAWN_MAX_POINTS) {
            showError(`At most ${SPAWN_MAX_POINTS} spawn points are supported.`);
            return;
        }
        spawnPlacing = true;
        map.getCanvas().style.cursor = 'crosshair';
        setSpawnButtonsActive(true);
        showInfo(`Click inside the polygon to place spawn point ${spawnExtras.length + 2} (Esc to cancel).`);
    }

    function stopSpawnPlacement() {
        spawnPlacing = false;
        if (map) map.getCanvas().style.cursor = '';
        setSpawnButtonsActive(false);
    }

    function setSpawnButtonsActive(active) {
        if (spawnMapButton) spawnMapButton.style.backgroundColor = active ? '#3bb2d0' : '';
        const addBtn = document.getElementById('setting-spawn-add-btn');
        addBtn.classList.toggle('active', active);
        addBtn.innerHTML = active
            ? '<i class="fas fa-crosshairs"></i> Click inside the polygon (Esc to cancel)'
            : '<i class="fas fa-plus"></i> Add spawn point';
    }

    function onSpawnMapClick(e) {
        if (!spawnPlacing) {
            // Without a polygon, a map click moves the red pin; keep the list in step
            updateSpawnListCoords();
            return;
        }
        const polygon = getSpawnPolygon();
        if (!polygon) {
            stopSpawnPlacement();
            return;
        }
        if (!isSpawnInside(e.lngLat, polygon)) {
            showError('Spawn points must be inside the polygon.');
            return;
        }
        addSpawnPoint(e.lngLat, 0);
        stopSpawnPlacement();
        warnSpawnSpacing();
        // The same click selects the polygon in MapboxDraw (or enters vertex editing
        // if it was already selected); deselect it so the next drag pans the map
        setTimeout(function () {
            if (draw.getMode() !== 'draw_polygon') draw.changeMode('simple_select', { featureIds: [] });
        }, 0);
    }

    function addSpawnPoint(lngLat, yawDeg) {
        const el = document.createElement('div');
        el.className = 'spawn-marker';
        const marker = new mapboxgl.Marker({ element: el, draggable: true })
            .setLngLat(lngLat)
            .addTo(map);
        const entry = { marker: marker, yawDeg: yawDeg || 0 };
        spawnExtras.push(entry);

        marker.on('drag', updateSpawnListCoords);
        marker.on('dragend', function () {
            refreshSpawnPoints();
            const polygon = getSpawnPolygon();
            if (polygon && !isSpawnInside(marker.getLngLat(), polygon)) {
                showError(`Spawn point ${spawnExtras.indexOf(entry) + 2} is outside the polygon. Move it back inside or right-click it to remove it.`);
            }
        });
        // Right-click removes; stop clicks reaching the map (placement, red pin moves)
        el.addEventListener('contextmenu', function (e) {
            e.preventDefault();
            e.stopPropagation();
            removeSpawnPoint(entry);
        });
        el.addEventListener('click', function (e) {
            e.stopPropagation();
        });
        releasePolygonOnPress(el);

        refreshSpawnPoints();
        return entry;
    }

    function removeSpawnPoint(entry) {
        entry.marker.remove();
        spawnExtras = spawnExtras.filter(function (e) { return e !== entry; });
        refreshSpawnPoints();
    }

    function clearSpawnPoints() {
        spawnExtras.forEach(function (e) { e.marker.remove(); });
        spawnExtras = [];
        spawnPrimaryYawDeg = 0;
        if (spawnPlacing) stopSpawnPlacement();
        refreshSpawnPoints();
    }

    // Renumber markers, flag the ones outside the polygon, label the red pin, redraw the list
    function refreshSpawnPoints() {
        const polygon = getSpawnPolygon();
        spawnExtras.forEach(function (entry, i) {
            const num = i + 2;
            const outside = polygon ? !isSpawnInside(entry.marker.getLngLat(), polygon) : false;
            const el = entry.marker.getElement();
            el.textContent = String(num);
            el.title = `Spawn point ${num}${outside ? ' (outside the polygon)' : ''}: drag to move, right-click to remove`;
            el.classList.toggle('spawn-marker--outside', outside);
        });
        updateSpawnPrimaryBadge();
        renderSpawnList();
    }

    // Number the red pin "1" while there are other spawn points
    function updateSpawnPrimaryBadge() {
        if (!centerMarker) return;
        const show = spawnExtras.length > 0;
        if (show && !spawnPrimaryBadge) {
            spawnPrimaryBadge = document.createElement('span');
            spawnPrimaryBadge.className = 'spawn-primary-badge';
            spawnPrimaryBadge.textContent = '1';
            centerMarker.getElement().appendChild(spawnPrimaryBadge);
            centerMarker.getElement().title = 'Spawn point 1 (world origin): drag to move';
        } else if (!show && spawnPrimaryBadge) {
            spawnPrimaryBadge.remove();
            spawnPrimaryBadge = null;
            centerMarker.getElement().removeAttribute('title');
        }
    }

    function getSpawnRows() {
        const rows = [];
        if (centerMarker) {
            rows.push({ num: 1, lngLat: centerMarker.getLngLat(), yawDeg: spawnPrimaryYawDeg, entry: null });
        }
        spawnExtras.forEach(function (entry, i) {
            rows.push({ num: i + 2, lngLat: entry.marker.getLngLat(), yawDeg: entry.yawDeg, entry: entry });
        });
        return rows;
    }

    function formatSpawnLngLat(lngLat) {
        return `${lngLat.lat.toFixed(5)}, ${lngLat.lng.toFixed(5)}`;
    }

    // Spawn point list in the settings panel: number, position, yaw, remove
    function renderSpawnList() {
        const list = document.getElementById('spawn-points-list');
        if (!list) return;
        const polygon = getSpawnPolygon();
        list.innerHTML = '';

        getSpawnRows().forEach(function (row) {
            const outside = polygon ? !isSpawnInside(row.lngLat, polygon) : false;
            const rowEl = document.createElement('div');
            rowEl.className = 'spawn-row' + (outside ? ' spawn-row--outside' : '');

            const badge = document.createElement('span');
            badge.className = 'spawn-row-badge' + (row.entry ? '' : ' spawn-row-badge--primary');
            badge.textContent = String(row.num);
            badge.title = row.entry ? `Spawn point ${row.num}` : 'Spawn point 1: world origin (red pin)';

            const coords = document.createElement('span');
            coords.className = 'spawn-row-coords';
            coords.textContent = formatSpawnLngLat(row.lngLat);
            coords.title = (outside ? 'Outside the polygon. ' : '') + 'Click to show on the map';
            coords.addEventListener('click', function () {
                const target = row.entry ? row.entry.marker.getLngLat() : centerMarker.getLngLat();
                map.easeTo({ center: target });
            });

            const yaw = document.createElement('input');
            yaw.type = 'number';
            yaw.className = 'spawn-row-yaw';
            yaw.step = '15';
            yaw.value = String(row.yawDeg);
            yaw.title = 'Yaw in degrees, counter-clockwise from east (0 = facing east, 90 = north)';
            yaw.addEventListener('change', function () {
                const value = parseFloat(this.value);
                const yawDeg = isFinite(value) ? value : 0;
                this.value = String(yawDeg);
                if (row.entry) {
                    row.entry.yawDeg = yawDeg;
                } else {
                    spawnPrimaryYawDeg = yawDeg;
                }
            });

            const remove = document.createElement('button');
            remove.type = 'button';
            remove.className = 'spawn-row-remove';
            remove.innerHTML = '<i class="fas fa-trash-alt"></i>';
            if (row.entry) {
                remove.title = `Remove spawn point ${row.num}`;
                remove.addEventListener('click', function () { removeSpawnPoint(row.entry); });
            } else {
                // Spawn 1 is the world origin and cannot be removed
                remove.disabled = true;
            }

            rowEl.appendChild(badge);
            rowEl.appendChild(coords);
            rowEl.appendChild(yaw);
            rowEl.appendChild(remove);
            list.appendChild(rowEl);
        });
    }

    // Update only the coordinates while a marker is dragged (keeps input focus)
    function updateSpawnListCoords() {
        const cells = document.querySelectorAll('#spawn-points-list .spawn-row-coords');
        getSpawnRows().forEach(function (row, i) {
            if (cells[i]) cells[i].textContent = formatSpawnLngLat(row.lngLat);
        });
    }

    // Called from validateForGeneration(): every additional spawn point must be inside the polygon
    function validateSpawnPoints(polygon) {
        refreshSpawnPoints();
        for (let i = 0; i < spawnExtras.length; i++) {
            if (!isSpawnInside(spawnExtras[i].marker.getLngLat(), polygon)) {
                showError(`Spawn point ${i + 2} must be inside the polygon. Drag it inside or right-click it to remove it.`);
                return false;
            }
        }
        warnSpawnSpacing();
        return true;
    }

    // Non-blocking warning when spawn points are close enough for robots or helipads to overlap
    function warnSpawnSpacing() {
        const rows = getSpawnRows();
        const minDist = config.includeHelipad ? SPAWN_HELIPAD_SIZE_M : 1;
        for (let i = 0; i < rows.length; i++) {
            for (let j = i + 1; j < rows.length; j++) {
                const d = turf.distance(
                    turf.point([rows[i].lngLat.lng, rows[i].lngLat.lat]),
                    turf.point([rows[j].lngLat.lng, rows[j].lngLat.lat]),
                    { units: 'kilometers' }
                ) * 1000;
                if (d < minDist) {
                    showInfo(`Spawn points ${rows[i].num} and ${rows[j].num} are only ${d.toFixed(1)} m apart`
                        + (config.includeHelipad ? `; their ${SPAWN_HELIPAD_SIZE_M} m helipads will overlap.` : '.'));
                    return;
                }
            }
        }
    }

    // Form payload: all spawn points, primary first. The server takes the primary's
    // position from launchLocation and validates the rest against the polygon.
    function getSpawnPointsPayload() {
        return getSpawnRows().map(function (row) {
            return { lng: row.lngLat.lng, lat: row.lngLat.lat, yawDeg: row.yawDeg };
        });
    }
    // ===== End multi-spawn =====

    // --- Ground friction (roadmap: "Friction components to the ground") ---
    // Presets live in index.html as <option data-mu="...">. 'default' sends no
    // coefficient to the world (Gazebo keeps mu = 1); any other choice makes the
    // server add the GroundFriction system plugin to the terrain model.

    function applyFrictionConfigToForm() {
        const presetSelect = document.getElementById('setting-friction-preset');
        const muInput = document.getElementById('setting-friction-coefficient');
        presetSelect.value = config.frictionPreset;
        if (presetSelect.value !== config.frictionPreset) {
            // Unknown preset saved by another version: fall back to Gazebo's default
            config.frictionPreset = 'default';
            config.frictionCoefficient = DEFAULT_CONFIG.frictionCoefficient;
            presetSelect.value = 'default';
        }
        muInput.value = config.frictionCoefficient;
        muInput.disabled = config.frictionPreset === 'default';
        updateFrictionNote();
    }

    function updateFrictionNote() {
        const note = document.getElementById('setting-friction-note');
        const isFortress = document.getElementById('setting-gazebo-version').value === 'fortress';
        if (config.frictionPreset === 'default') {
            note.style.display = 'none';
            return;
        }
        note.textContent = isFortress
            ? 'Not supported for Fortress: the world keeps Gazebo\'s default friction.'
            : 'Gazebo ignores friction set on heightmaps, so the world loads the GroundFriction plugin. ' +
              'Build it once from gazebo_plugins/ground_friction, or Gazebo keeps mu = 1.';
        note.style.display = '';
    }

    function setupFrictionSettings() {
        const presetSelect = document.getElementById('setting-friction-preset');
        const muInput = document.getElementById('setting-friction-coefficient');

        presetSelect.addEventListener('change', function () {
            config.frictionPreset = this.value;
            const presetMu = this.selectedOptions[0] && this.selectedOptions[0].dataset.mu;
            if (presetMu !== undefined) {
                config.frictionCoefficient = parseFloat(presetMu);
            }
            applyFrictionConfigToForm();
            saveConfig();
        });

        muInput.addEventListener('change', function () {
            const mu = parseFloat(this.value);
            if (!Number.isFinite(mu) || mu < 0) {
                this.value = config.frictionCoefficient;
                return;
            }
            config.frictionCoefficient = mu;
            const selected = presetSelect.selectedOptions[0];
            if (!selected || selected.dataset.mu === undefined || parseFloat(selected.dataset.mu) !== mu) {
                config.frictionPreset = 'custom';
            }
            applyFrictionConfigToForm();
            saveConfig();
        });

        // The note depends on the target Gazebo version
        document.getElementById('setting-gazebo-version').addEventListener('change', updateFrictionNote);
    }

    // --- Smooth roads (roadmap feature) ---------------------------------------
    // The server flattens road corridors in the heightmap and smooths them along
    // their length. Settings: roadsSmooth (on/off), roadsSmoothingLength and
    // roadsShoulderWidth (metres), posted to /start-download and /end-download.

    function applyRoadsConfigToForm() {
        document.getElementById('setting-roads-smooth').checked = config.roadsSmooth;
        document.getElementById('setting-roads-smoothing-length').value = config.roadsSmoothingLength;
        document.getElementById('setting-roads-shoulder-width').value = config.roadsShoulderWidth;
        document.getElementById('setting-roads-options').style.display = config.roadsSmooth ? '' : 'none';
    }

    function setupRoadsSettings() {
        document.getElementById('setting-roads-smooth').addEventListener('change', function () {
            config.roadsSmooth = this.checked;
            document.getElementById('setting-roads-options').style.display = this.checked ? '' : 'none';
            saveConfig();
        });

        document.getElementById('setting-roads-smoothing-length').addEventListener('change', function () {
            const value = parseFloat(this.value);
            config.roadsSmoothingLength = value >= 10 ? Math.min(value, 1000) : DEFAULT_CONFIG.roadsSmoothingLength;
            this.value = config.roadsSmoothingLength;
            saveConfig();
        });

        document.getElementById('setting-roads-shoulder-width').addEventListener('change', function () {
            const value = parseFloat(this.value);
            config.roadsShoulderWidth = value >= 0 ? Math.min(value, 100) : DEFAULT_CONFIG.roadsShoulderWidth;
            this.value = config.roadsShoulderWidth;
            saveConfig();
        });
    }

    function appendRoadsSettings(formData) {
        formData.append('roadsSmooth', config.roadsSmooth);
        formData.append('roadsSmoothingLength', config.roadsSmoothingLength);
        formData.append('roadsShoulderWidth', config.roadsShoulderWidth);
    }

    // --- end smooth roads -------------------------------------------------------

    // --- Tile utilities ---

    function long2tile(lon, zoom) {
        return Math.floor((lon + 180) / 360 * Math.pow(2, zoom));
    }

    function lat2tile(lat, zoom) {
        return Math.floor((1 - Math.log(Math.tan(lat * Math.PI / 180) + 1 / Math.cos(lat * Math.PI / 180)) / Math.PI) / 2 * Math.pow(2, zoom));
    }

    function tile2long(x, z) {
        return x / Math.pow(2, z) * 360 - 180;
    }

    function tile2lat(y, z) {
        const n = Math.PI - 2 * Math.PI * y / Math.pow(2, z);
        return 180 / Math.PI * Math.atan(0.5 * (Math.exp(n) - Math.exp(-n)));
    }

    function getTileRect(x, y, zoom) {
        return [
            [tile2long(x, zoom), tile2lat(y + 1, zoom)], // SW
            [tile2long(x + 1, zoom), tile2lat(y + 1, zoom)], // SE
            [tile2long(x + 1, zoom), tile2lat(y, zoom)],     // NE
            [tile2long(x, zoom), tile2lat(y, zoom)],     // NW
            [tile2long(x, zoom), tile2lat(y + 1, zoom)]  // SW close
        ];
    }

    function getGrid(zoomLevel) {
        const features = draw.getAll().features;
        if (features.length === 0) return [];

        const polygon = features[0];
        const coords = polygon.geometry.coordinates[0];

        // Get bounding box of polygon
        const lngs = coords.map(c => c[0]);
        const lats = coords.map(c => c[1]);
        const minLng = Math.min(...lngs), maxLng = Math.max(...lngs);
        const minLat = Math.min(...lats), maxLat = Math.max(...lats);

        const TY = lat2tile(maxLat, zoomLevel);
        const BY = lat2tile(minLat, zoomLevel);
        const LX = long2tile(minLng, zoomLevel);
        const RX = long2tile(maxLng, zoomLevel);

        const rects = [];
        for (let y = TY; y <= BY; y++) {
            for (let x = LX; x <= RX; x++) {
                const tileRect = getTileRect(x, y, zoomLevel);
                const tilePolygon = turf.polygon([tileRect]);
                if (!turf.booleanDisjoint(tilePolygon, polygon)) {
                    rects.push(tileRect);
                }
            }
        }
        return rects;
    }

    function setGridVisible(visible, button) {
        gridVisible = visible;
        const btn = button || window._gridButton;
        if (btn) btn.style.backgroundColor = gridVisible ? '#3bb2d0' : '';
        if (gridVisible) {
            showGrid();
        } else {
            hideGrid();
        }
    }

    function showGrid() {
        if (draw.getAll().features.length === 0) {
            showError('Draw a polygon first before previewing the grid.');
            setGridVisible(false);
            return;
        }

        hideGrid();

        const grid = getGrid(config.zoomLevel);

        if (grid.length === 0) {
            showError('No tiles found in selection.');
            setGridVisible(false);
            return;
        }

        const gridFeatures = grid.map(rect => turf.polygon([rect]));

        map.addSource(GRID_LAYER_ID, {
            type: 'geojson',
            data: turf.featureCollection(gridFeatures)
        });

        map.addLayer({
            id: GRID_LAYER_ID,
            type: 'line',
            source: GRID_LAYER_ID,
            paint: {
                'line-color': '#ffffff',
                'line-width': 1,
                'line-opacity': 0.45
            }
        });

        //showInfo(`Grid preview: ${grid.length} tiles at zoom level ${GRID_ZOOM_LEVEL}`);
    }

    function hideGrid() {
        if (map.getSource(GRID_LAYER_ID)) {
            map.removeLayer(GRID_LAYER_ID);
            map.removeSource(GRID_LAYER_ID);
        }
    }

    // Show error message
    function showError(message) {
        console.error(message);
        Toastify({
            text: message,
            duration: 5000,
            gravity: "top",
            position: "center",
            style: { background: "linear-gradient(to right, #e74c3c, #c0392b)" },
            stopOnFocus: true
        }).showToast();
    }

    // Show success message
    function showSuccess(message) {
        console.log(message);
        Toastify({
            text: message,
            duration: 3000,
            gravity: "top",
            position: "center",
            style: { background: "linear-gradient(to right, #27ae60, #1e8449)" },
            stopOnFocus: true
        }).showToast();
    }

    // Show info message
    function showInfo(message) {
        console.log(message);
        Toastify({
            text: message,
            duration: 3000,
            gravity: "top",
            position: "center",
            style: { background: "linear-gradient(to right, #3498db, #2980b9)" },
            stopOnFocus: true
        }).showToast();
    }

    // Start the application when DOM is ready
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init);
    } else {
        init();
    }

})();
