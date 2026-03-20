function p = autosimReadPhysicsFromDroneFiles(p)
    if ~isstruct(p)
        return;
    end

    autoLoad = true;
    if isfield(p, 'auto_load_from_drone_files')
        autoLoad = logical(p.auto_load_from_drone_files);
    end
    if ~autoLoad
        return;
    end

    preferFile = true;
    if isfield(p, 'prefer_file_values')
        preferFile = logical(p.prefer_file_values);
    end

    massFile = nan;
    maxForceFile = nan;
    areaFile = nan;
    cdFile = nan;
    rhoFile = nan;
    landingFactorFile = nan;

    yamlPath = autosimWindStringField(p, 'drone_yaml_path', "");
    if strlength(yamlPath) > 0 && isfile(char(yamlPath))
        maxForceFile = autosimReadYamlScalar(char(yamlPath), 'maxForce');
        massFile = autosimFirstFinite([ ...
            autosimReadYamlScalar(char(yamlPath), 'mass_kg'), ...
            autosimReadYamlScalar(char(yamlPath), 'mass')]);
        areaFile = autosimFirstFinite([ ...
            autosimReadYamlScalar(char(yamlPath), 'frontal_area_m2'), ...
            autosimReadYamlScalar(char(yamlPath), 'drag_area_m2')]);
        cdFile = autosimFirstFinite([ ...
            autosimReadYamlScalar(char(yamlPath), 'drag_coefficient'), ...
            autosimReadYamlScalar(char(yamlPath), 'cd')]);
        rhoFile = autosimReadYamlScalar(char(yamlPath), 'air_density_kgpm3');
        landingFactorFile = autosimReadYamlScalar(char(yamlPath), 'landing_limit_factor');
    end

    urdfPath = autosimWindStringField(p, 'drone_urdf_path', "");
    if ~isfinite(massFile) && strlength(urdfPath) > 0 && isfile(char(urdfPath))
        massFile = autosimReadXmlAttributeScalar(char(urdfPath), '<mass', 'value');
    end

    sdfPath = autosimWindStringField(p, 'drone_sdf_path', "");
    if (~isfinite(massFile) || ~isfinite(maxForceFile)) && strlength(sdfPath) > 0 && isfile(char(sdfPath))
        if ~isfinite(massFile)
            massFile = autosimReadXmlTagScalar(char(sdfPath), 'mass');
        end
        if ~isfinite(maxForceFile)
            maxForceFile = autosimReadXmlTagScalar(char(sdfPath), 'maxForce');
        end
    end

    p.mass_kg = autosimMergePhysicsValue(p, 'mass_kg', massFile, preferFile, 1.4);
    p.max_total_thrust_n = autosimMergePhysicsValue(p, 'max_total_thrust_n', maxForceFile, preferFile, 24.0);
    p.frontal_area_m2 = autosimMergePhysicsValue(p, 'frontal_area_m2', areaFile, preferFile, 0.075);
    p.drag_coefficient = autosimMergePhysicsValue(p, 'drag_coefficient', cdFile, preferFile, 1.10);
    p.air_density_kgpm3 = autosimMergePhysicsValue(p, 'air_density_kgpm3', rhoFile, preferFile, 1.225);
    p.landing_limit_factor = autosimMergePhysicsValue(p, 'landing_limit_factor', landingFactorFile, preferFile, 0.5);

    p.source_mass = autosimPhysicsSourceLabel(massFile, urdfPath, sdfPath, 'mass');
    p.source_max_force = autosimPhysicsSourceLabel(maxForceFile, yamlPath, sdfPath, 'maxForce');
end


